#!/usr/bin/env python3

#
# This file is part of TriXium.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""TriXium on the Xilinx VC707 (XC7VX485T-2, one 64-bit DDR3 SODIMM, 1 GB MT8JTF12864).

A port of the YPCB-00338 target (trixium_ypcb.py) to a single DDR3 controller and no host
link: the console is the board's USB-UART and the weights come from the SD card (the
firmware's `sdload` reads the BONS image with FatFs and pokes it into the loader, the CSR
path the PCIe host tools use). The DDR3 sits on HP banks: LiteDRAM's V7DDRPHY with ODELAY
write leveling, DDR3-800 in a 100 MHz ddr domain by default (the rate this board's other
LiteX designs run at; --ddr-clk-freq 125e6 for DDR3-1000, the upstream litex-boards target's).

1 GB leaves room for two engine channels of the Ternary-Bonsai 1.7B without the int16
embedding table (convert --board vc707: 2 windows of 352 MB above the 320 MB weight base,
~269 MB used in each).

From the repository root:
  ./trixium_vc707.py --build                         # -> build/vc707/ + build/vc707.verdict
  ./trixium_vc707.py --load                          # JTAG

Vivado 2020.1 (--vivado-settings, the board's licensed install).
"""

import os
import sys
import argparse

from migen import *

from litex.gen import *

from litex_boards.platforms import xilinx_vc707

from litex.soc.cores.clock import *
from litex.soc.cores.led import LedChaser
from litex.soc.integration.soc import *
from litex.soc.integration.builder import *
from litex.soc.interconnect.axi import AXIInterface

from litedram.modules import MT8JTF12864
from litedram.core.controller import ControllerSettings
from litedram.phy.s7ddrphy import V7DDRPHY

from model.layout import HBM_WEIGHT_BASE, HDR_CPU_OFFSET, ddr_chan_stride
from gateware.core.core import TriXiumConfig, TriXiumCore
from gateware.memory.ddr import add_ddr_domain_controllers, engine_ddr_port
from gateware.common import add_cluster_constraints, add_zbb_constant
from gateware.common import set_main_ram_executable
from gateware.common import set_vivado_env, write_verdict

VIVADO_2020 = "/opt/Xilinx/Vivado/2020.1"

# The SODIMM has DM pins: the controller writes partial words with byte masks, the engine
# ports need no read-modify-write.
ENGINE_RMW = False

def ddr_controller_settings(read_time=256, write_time=128, refresh_postponing=1):
    """The YPCB's controller settings (trixium_ypcb.py: weight streams are reads)."""
    return lambda: ControllerSettings(
        read_time          = read_time,
        write_time         = write_time,
        refresh_postponing = refresh_postponing)

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    """Clocks: `sys` (CPU, TriXium core), `ddr` + `ddr4x` (the LiteDRAM controller and the
    V7DDRPHY's serdes, DDR3 rate = 8 x ddr) and the 200 MHz IDELAY reference, from one MMCM
    on the 200 MHz oscillator (CPU_RESET button resets it)."""
    def __init__(self, platform, sys_clk_freq, ddr_clk_freq):
        assert ddr_clk_freq != sys_clk_freq
        self.rst       = Signal()
        self.cd_sys    = ClockDomain()
        self.cd_ddr    = ClockDomain()
        self.cd_ddr4x  = ClockDomain()
        self.cd_idelay = ClockDomain()

        # # #

        self.pll = pll = S7MMCM(speedgrade=-2)
        self.comb += pll.reset.eq(platform.request("cpu_reset") | self.rst)
        pll.register_clkin(platform.request("clk200"), 200e6)
        # sys first: CLKOUT0 is the MMCM's fractional output (unused: 120 MHz = 1200 MHz / 10; 125 MHz would need /9.6, off the 0.125 grid).
        pll.create_clkout(self.cd_sys,    sys_clk_freq,   margin=1e-4)
        pll.create_clkout(self.cd_ddr,    ddr_clk_freq,   margin=1e-4)
        pll.create_clkout(self.cd_ddr4x,  4*ddr_clk_freq, margin=1e-4)
        pll.create_clkout(self.cd_idelay, 200e6,          margin=1e-4)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.
        platform.add_false_path_constraints(self.cd_sys.clk, self.cd_ddr.clk)

        self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

# TriXiumVC707SoC ----------------------------------------------------------------------------------

# The controller's (sdram, ddrphy, sdram_generator, sdram_checker) CSR pages, in the ddr domain
# behind a wishbone crossing (memory/ddr.py), as on the YPCB.
DDR_CSR_WINDOW = 0xF0020000
DDR_CSR_STRIDE = 0x2000

class TriXiumVC707SoC(SoCCore):
    def __init__(self, sys_clk_freq=120e6, ddr_clk_freq=100e6, engine_channels=2,
        main_ram_size  = 0x1000_0000,
        chan_stride    = None,
        chan_skew      = None,
        read_time      = 256,
        write_time     = 128,
        refresh_postponing = 1,
        trit5          = True,
        prefetch       = 2048,
        vec_fused      = True,
        **kwargs):
        chan_mod = engine_channels
        assert chan_mod & (chan_mod - 1) == 0, "engine channels must be a power of two"
        platform = xilinx_vc707.Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, ddr_clk_freq=ddr_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        kwargs["integrated_sram_size"] = max(kwargs.get("integrated_sram_size") or 0, 0x4000)
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident = "TriXium on VC707", **kwargs)
        self.add_constant("TRIXIUM_VARIANT", ord("b"))

        # DDR3 -------------------------------------------------------------------------------------
        # One controller: the CPU's main_ram (256 MB, through the L2 cache) and the engine ports.
        self.sdrams = []
        phy = V7DDRPHY(platform.request("ddram"),
            memtype      = "DDR3",
            sys_clk_freq = ddr_clk_freq,
            name         = "ddrphy")
        phy.add_constraints(platform) if hasattr(phy, "add_constraints") else None
        add_ddr_domain_controllers(self, [phy], [MT8JTF12864(ddr_clk_freq, "1:4")], ddr_clk_freq,
            main_ram_size, kwargs.get("l2_size", 0), DDR_CSR_WINDOW,
            phy_renames={"sys": "ddr", "sys4x": "ddr4x"},
            controller_settings=ddr_controller_settings(read_time, write_time, refresh_postponing),
            with_bist=True)
        # One controller: the BIOS's standard init (write leveling) levels it; no
        # SDRAM_CSR_STRIDE/SDRAM_CONTROLLERS (those select the multi-controller CK phase path).
        set_main_ram_executable(self)

        # TriXium Memory Ports ---------------------------------------------------------------------
        memory_ports = []
        for c in range(engine_channels):
            axi = AXIInterface(data_width=256, address_width=32, id_width=6)
            for j, m in enumerate(engine_ddr_port(self.sdrams[0], axi, clock_domain="sys_port",
                                                  rmw=ENGINE_RMW)):
                setattr(self, f"engport{c}_{j}", m)
            memory_ports.append(axi)

        # SD card (weights, firmware) --------------------------------------------------------------
        self.add_sdcard()

        # Leds -------------------------------------------------------------------------------------
        self.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq)

        # TriXium ----------------------------------------------------------------------------------
        # The YPCB's window scheme on one 1 GB controller: above HBM_WEIGHT_BASE (320 MB) split
        # evenly between the channels (352 MB each at 2), skewed by 64 KB / windows so the
        # streams sit in different banks (LiteDRAM ROW_BANK_COL: bank in byte address bits
        # 13..15 on a 64-bit channel). main_ram (0..256 MB) and the BONS staging MB sit below.
        ctrl_size = 1 << 30
        if chan_stride is None:
            chan_stride = ddr_chan_stride(ctrl_size, chan_mod)
        if chan_skew is None:
            chan_skew = 0x10000//chan_mod
        chan_window = chan_stride - (chan_mod - 1)*chan_skew
        assert chan_stride % 0x10000 == 0
        assert HBM_WEIGHT_BASE + chan_mod*chan_stride <= ctrl_size
        assert chan_skew % 0x1000 == 0 and (chan_mod - 1)*chan_skew < chan_stride // 64
        assert main_ram_size <= HBM_WEIGHT_BASE - 0x100000

        config = TriXiumConfig(scratchpad_xpm=True,
            chan_stride       = chan_stride,
            chan_mod          = chan_mod,
            chan_skew         = chan_skew,
            with_trit5        = trit5,
            with_prefetch     = bool(prefetch),
            reader_fifo_depth = prefetch or 512,
            with_vec_fused    = vec_fused)
        self.trixium = TriXiumCore(memory_ports, config)
        for name, value in self.trixium.constants.items():
            self.add_constant(name, value)
        add_cluster_constraints(platform)

        self.bus.add_slave("scratchpad", self.trixium.bus,
            SoCRegion(origin=0xa000_0000, size=self.trixium.scratchpad.size, cached=False))

        # No DMA source: uploads use the loader's CSR poke path (firmware sdload).
        self.comb += self.trixium.upload.valid.eq(0)

        # Board memory map, for the firmware.
        self.add_constant("TRIXIUM_SDLOAD", 1)
        self.add_constant("TRIXIUM_LOADER_ADDS_CHAN_OFFSET", 1)
        self.add_constant("TRIXIUM_HDR_ADDR",         0x4000_0000 + HDR_CPU_OFFSET)
        self.add_constant("TRIXIUM_CHAN_WEIGHT_BASE", HBM_WEIGHT_BASE)
        self.add_constant("TRIXIUM_CHAN_STRIDE",      chan_stride)
        self.add_constant("TRIXIUM_CHAN_WINDOW",      chan_window)
        self.add_constant("TRIXIUM_N_CHANNELS",       engine_channels)
        self.add_constant("TRIXIUM_CHAN_MOD",         chan_mod)
        self.add_constant("TRIXIUM_CHAN_SKEW",        chan_skew)
        self.add_constant("TRIXIUM_MEM_CPU_OFFSET",   0)
        self.add_constant("TRIXIUM_N_MEMCTRL",        1)
        self.add_constant("TRIXIUM_DDR_CSR_STRIDE",   DDR_CSR_STRIDE)

# Build --------------------------------------------------------------------------------------------

# CPU: the YPCB's (32 KB I$ + 32 KB D$, branch prediction, Zcbm, Zbb).
VEXII_ARGS = ("--fetch-l1-ways=4 --fetch-l1-sets=128 --lsu-l1-ways=4 --lsu-l1-sets=128 "
    "--with-btb --with-ras --with-gshare --with-rvZcbm --with-rvZbb")

def main():
    if os.environ.get("PYTHONHASHSEED") != "0":
        os.environ["PYTHONHASHSEED"] = "0"
        os.execv(sys.executable, [sys.executable] + sys.argv)

    from litex.build.parser import LiteXArgumentParser
    from software.lab.timing import gate
    parser = LiteXArgumentParser(platform=xilinx_vc707.Platform, description="TriXium on the Xilinx VC707.")
    parser.add_target_argument("--sys-clk-freq",       default=120e6,    type=float,                          help="System clock frequency (CPU, TriXium core).")
    parser.add_target_argument("--ddr-clk-freq",       default=100e6,    type=float,                          help="DDR3 controller clock frequency (DDR3 rate = 8x: 100e6 = DDR3-800).")
    parser.add_target_argument("--engine-channels",    default=2,        type=int,                            help="Engine channels (power of two).")
    parser.add_target_argument("--read-time",          default=256,      type=int,                            help="LiteDRAM read time.")
    parser.add_target_argument("--write-time",         default=128,      type=int,                            help="LiteDRAM write time.")
    parser.add_target_argument("--refresh-postponing", default=1,        type=int,                            help="LiteDRAM refresh postponing (1-8).")
    parser.add_target_argument("--chan-stride",        default=None,     type=lambda x: int(x, 0),            help="Engine window stride (default: the controller above the weight base split evenly).")
    parser.add_target_argument("--trit5",              action=argparse.BooleanOptionalAction, default=True,  help="Unpackers for 5-trits-per-byte weights.")
    parser.add_target_argument("--prefetch",           default=2048,     type=int,                            help="Weight prefetch: reader FIFO beats per channel (0: none).")
    parser.add_target_argument("--vec-fused",          action=argparse.BooleanOptionalAction, default=True,  help="Vector unit with the fused clamp/round instructions.")
    parser.add_target_argument("--vivado-settings",    default=os.environ.get("LITEX_ENV_VIVADO", VIVADO_2020), help="Vivado install sourced by the build (settings64.sh).")
    parser.set_defaults(
        cpu_type                             = "vexiiriscv",
        update_repo                          = "no",
        vexii_args                           = VEXII_ARGS,
        uart_baudrate                        = 115200,
        l2_size                              = 0, # SD DMA straight to DRAM (memory/ddr.py).
        vivado_synth_directive               = "PerformanceOptimized",
        vivado_opt_directive                 = "ExploreWithRemap",
        vivado_place_directive               = "ExtraTimingOpt",
        vivado_post_place_phys_opt_directive = "AggressiveExplore",
        vivado_route_directive               = "AggressiveExplore",
        vivado_post_route_phys_opt_directive = "AggressiveExplore",
        vivado_max_threads                   = 4,
    )
    args = parser.parse_args()

    soc = TriXiumVC707SoC(
        sys_clk_freq       = args.sys_clk_freq,
        ddr_clk_freq       = args.ddr_clk_freq,
        engine_channels    = args.engine_channels,
        read_time          = args.read_time,
        write_time         = args.write_time,
        refresh_postponing = args.refresh_postponing,
        trit5              = args.trit5,
        prefetch           = args.prefetch,
        vec_fused          = args.vec_fused,
        chan_stride        = args.chan_stride,
        **parser.soc_argdict)
    add_zbb_constant(soc, args)

    builder_argd = parser.builder_argdict
    if builder_argd.get("output_dir") is None:
        builder_argd["output_dir"] = os.path.join("build", "vc707")
    if builder_argd.get("csr_csv") is None:
        builder_argd["csr_csv"] = os.path.join(builder_argd["output_dir"], "csr.csv")
    builder = Builder(soc, **builder_argd)
    if args.build:
        set_vivado_env(args.vivado_settings)
        verdict = builder.output_dir.rstrip("/") + ".verdict"
        try:
            builder.build(**parser.toolchain_argdict)
        except OSError:
            if builder.compile_gateware:
                write_verdict(verdict, 1)
            raise

        if builder.compile_gateware:
            report = os.path.join(builder.gateware_dir, f"{soc.platform.name}_timing.rpt")
            print("\nTiming gate:")
            rc = gate(report, strict=True, board="vc707", clocks=[round(1e9/args.sys_clk_freq, 3)])
            write_verdict(verdict, rc)
            if rc:
                sys.exit(rc)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
