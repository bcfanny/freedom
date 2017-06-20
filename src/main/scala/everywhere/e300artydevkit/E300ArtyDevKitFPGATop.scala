// See LICENSE for license details.
package sifive.freedom.everywhere.e300artydevkit

import Chisel._
import chisel3.core.{Input, Output, attach, Param, IntParam}
import chisel3.experimental.{RawModule, Analog, withClockAndReset, withClock}
import diplomacy.{LazyModule}
import config._

import sifive.blocks.devices.gpio.{GPIOPin}
import sifive.blocks.devices.spi.{PeripherySPIFlashKey, SPIGPIOPort}

import sifive.freedom.ip.xilinx.{IBUFG, IOBUF, PULLUP, mmcm, reset_sys, PowerOnResetFPGAOnly, ClockDivider256}

//-------------------------------------------------------------------------
// E300ArtyDevKitFPGATop
//-------------------------------------------------------------------------

class E300ArtyDevKitFPGATop(implicit val p: Parameters) extends RawModule {

  //-----------------------------------------------------------------------
  // Interface
  //-----------------------------------------------------------------------

  // Clock & Reset
  val CLK100MHZ    = IO(Input(Clock()))
  val ck_rst       = IO(Input(Bool()))

  // Green LEDs
  val led_0        = IO(Output(Bool()))
  val led_1        = IO(Output(Bool()))
  val led_2        = IO(Output(Bool()))
  val led_3        = IO(Analog(1.W))

  // RGB LEDs, 3 pins each
  val led0_r       = IO(Output(Bool()))
  val led0_g       = IO(Output(Bool()))
  val led0_b       = IO(Output(Bool()))

  val led1_r       = IO(Output(Bool()))
  val led1_g       = IO(Output(Bool()))
  val led1_b       = IO(Output(Bool()))

  val led2_r       = IO(Output(Bool()))
  val led2_g       = IO(Output(Bool()))
  val led2_b       = IO(Output(Bool()))

  // Sliding switches, 3 used as GPIO
  // sw_3 selects input to UART0
  val sw_0         = IO(Input(Bool()))
  val sw_1         = IO(Input(Bool()))
  val sw_2         = IO(Input(Bool()))
  val sw_3         = IO(Input(Bool()))

  // Buttons. First 3 used as GPIO, the last is used as wakeup
  val btn_0        = IO(Analog(1.W))
  val btn_1        = IO(Analog(1.W))
  val btn_2        = IO(Analog(1.W))
  val btn_3        = IO(Analog(1.W))

  // Dedicated QSPI interface
  val qspi_cs      = IO(Output(Bool()))
  val qspi_sck     = IO(Output(Bool()))
  val qspi_dq_0    = IO(Analog(1.W))
  val qspi_dq_1    = IO(Analog(1.W))
  val qspi_dq_2    = IO(Analog(1.W))
  val qspi_dq_3    = IO(Analog(1.W))

  // UART0 (GPIO 16,17)
  val uart_rxd_out = IO(Output(Bool()))
  val uart_txd_in  = IO(Analog(1.W))

  // UART1 (GPIO 24,25) (not present on 48-pin)
  val ja_0         = IO(Analog(1.W))
  val ja_1         = IO(Analog(1.W))

  // Arduino (aka chipkit) shield digital IO pins, 14 is not connect to the
  // chip, used for debug
  val ck_io        = IO(Vec(20, Analog(1.W)))

  // Dedicated SPI pins on 6 pin header standard on later arduino models
  // connect to SPI2 (on FPGA)
  val ck_miso      = IO(Analog(1.W))
  val ck_mosi      = IO(Analog(1.W))
  val ck_ss        = IO(Analog(1.W))
  val ck_sck       = IO(Analog(1.W))

  // JD (used for JTAG connection)
  val jd_0         = IO(Analog(1.W))    // TDO
  val jd_1         = IO(Analog(1.W))    // TRST_n
  val jd_2         = IO(Input(Clock())) // TCK
  val jd_4         = IO(Analog(1.W))    // TDI
  val jd_5         = IO(Analog(1.W))    // TMS
  val jd_6         = IO(Analog(1.W))    // SRST_n

  //-----------------------------------------------------------------------
  // Wire declrations
  //-----------------------------------------------------------------------

  // Note: these frequencies are approximate.
  val clock_8MHz     = Wire(Clock())
  val clock_32MHz    = Wire(Clock())
  val clock_65MHz    = Wire(Clock())

  val mmcm_locked    = Wire(Bool())

  val slow_clock     = Wire(Bool())

  val reset_core     = Wire(Bool())
  val reset_bus      = Wire(Bool())
  val reset_periph   = Wire(Bool())
  val reset_intcon_n = Wire(Bool())
  val reset_periph_n = Wire(Bool())

  val SRST_n         = Wire(Bool())

  //-----------------------------------------------------------------------
  // Clock & Reset
  //-----------------------------------------------------------------------

  // mixed-mode clock generator
  val ip_mmcm = Module(new mmcm())
  ip_mmcm.io.clk_in1 := CLK100MHZ
  clock_8MHz         := ip_mmcm.io.clk_out1  // 8.388 MHz = 32.768 kHz * 256
  clock_65MHz        := ip_mmcm.io.clk_out2  // 65 Mhz
  clock_32MHz        := ip_mmcm.io.clk_out3  // 65/2 Mhz
  ip_mmcm.io.resetn  := ck_rst
  mmcm_locked        := ip_mmcm.io.locked

  // Divide clock by 256, used to generate 32.768 kHz clock for AON block
  slow_clock := ClockDivider256(clock_8MHz, ~mmcm_locked)

  // processor system reset module
  val ip_reset_sys = Module(new reset_sys())
  ip_reset_sys.io.slowest_sync_clk := clock_8MHz
  ip_reset_sys.io.ext_reset_in     := ck_rst & SRST_n
  ip_reset_sys.io.aux_reset_in     := true.B
  ip_reset_sys.io.mb_debug_sys_rst := false.B
  ip_reset_sys.io.dcm_locked       := mmcm_locked
  reset_core                       := ip_reset_sys.io.mb_reset
  reset_bus                        := ip_reset_sys.io.bus_struct_reset
  reset_periph                     := ip_reset_sys.io.peripheral_reset
  reset_intcon_n                   := ip_reset_sys.io.interconnect_aresetn
  reset_periph_n                   := ip_reset_sys.io.peripheral_aresetn

  // jtag reset
  val jtag_power_on_reset = PowerOnResetFPGAOnly(clock_32MHz)

  //-----------------------------------------------------------------------
  // DUT
  //-----------------------------------------------------------------------

  withClockAndReset(clock_32MHz, ck_rst) {
    val dut = Module(new E300ArtyDevKitTop)

    //---------------------------------------------------------------------
    // SPI flash IOBUFs
    //---------------------------------------------------------------------

    qspi_sck := dut.io.pads.qspi.sck.o.oval
    qspi_cs  := dut.io.pads.qspi.cs(0).o.oval

    IOBUF(qspi_dq_0, dut.io.pads.qspi.dq(0))
    IOBUF(qspi_dq_1, dut.io.pads.qspi.dq(1))
    IOBUF(qspi_dq_2, dut.io.pads.qspi.dq(2))
    IOBUF(qspi_dq_3, dut.io.pads.qspi.dq(3))

    //---------------------------------------------------------------------
    // JTAG IOBUFs
    //---------------------------------------------------------------------

    dut.io.pads.jtag.TCK.i.ival := IBUFG(jd_2).asUInt

    IOBUF(jd_5, dut.io.pads.jtag.TMS)
    PULLUP(jd_5)

    IOBUF(jd_4, dut.io.pads.jtag.TDI)
    PULLUP(jd_4)

    IOBUF(jd_0, dut.io.pads.jtag.TDO)

    IOBUF(jd_1, dut.io.pads.jtag.TRSTn.get)
    PULLUP(jd_1)

    // mimic putting a pullup on this line (part of reset vote)
    SRST_n := IOBUF(jd_6)
    PULLUP(jd_6)

    // jtag reset
    dut.io.jtag_reset := jtag_power_on_reset

    //---------------------------------------------------------------------
    // Assignment to package pins
    //---------------------------------------------------------------------
    // Pins IO0-IO13
    //
    // FTDI UART TX/RX are not connected to ck_io[0,1]
    // the way they are on Arduino boards.  We copy outgoing
    // data to both places, switch 3 (sw[3]) determines whether
    // input to UART comes from FTDI chip or gpio_16 (shield pin PD0)

    val iobuf_ck0 = Module(new IOBUF())
    iobuf_ck0.io.I := dut.io.pads.gpio(16).o.oval
    iobuf_ck0.io.T := ~dut.io.pads.gpio(16).o.oe
    attach(iobuf_ck0.io.IO, ck_io(0))   // UART0 RX

    val iobuf_uart_txd = Module(new IOBUF())
    iobuf_uart_txd.io.I := dut.io.pads.gpio(16).o.oval
    iobuf_uart_txd.io.T := ~dut.io.pads.gpio(16).o.oe
    attach(iobuf_uart_txd.io.IO, uart_txd_in)

    // gpio(16) input is shared between FTDI TX pin and the Arduino shield pin using SW[3]
    dut.io.pads.gpio(16).i.ival := Mux(sw_3, iobuf_ck0.io.O & dut.io.pads.gpio(16).o.ie, iobuf_uart_txd.io.O & dut.io.pads.gpio(16).o.ie)

    val iobuf_uart_rxd = Module(new IOBUF())
    iobuf_uart_rxd.io.I := dut.io.pads.gpio(17).o.oval
    iobuf_uart_rxd.io.T := ~dut.io.pads.gpio(17).o.oe
    uart_rxd_out        := dut.io.pads.gpio(17).o.oval & dut.io.pads.gpio(17).o.oe

    // Shield header row 0: PD2-PD7
    IOBUF(ck_io(2),  dut.io.pads.gpio(18))
    IOBUF(ck_io(3),  dut.io.pads.gpio(19)) // PWM1(1)
    IOBUF(ck_io(4),  dut.io.pads.gpio(20)) // PWM1(0)
    IOBUF(ck_io(5),  dut.io.pads.gpio(21)) // PWM1(2)
    IOBUF(ck_io(6),  dut.io.pads.gpio(22)) // PWM1(3)
    IOBUF(ck_io(7),  dut.io.pads.gpio(23))

    // Header row 1: PB0-PB5
    IOBUF(ck_io(8),  dut.io.pads.gpio(0))  // PWM0(0)
    IOBUF(ck_io(9),  dut.io.pads.gpio(1))  // PWM0(1)
    IOBUF(ck_io(10), dut.io.pads.gpio(2))  // SPI CS(0) / PWM0(2)
    IOBUF(ck_io(11), dut.io.pads.gpio(3))  // SPI MOSI  / PWM0(3)
    IOBUF(ck_io(12), dut.io.pads.gpio(4))  // SPI MISO
    IOBUF(ck_io(13), dut.io.pads.gpio(5))  // SPI SCK

    dut.io.pads.gpio(6).i.ival  := 0.U
    dut.io.pads.gpio(7).i.ival  := 0.U
    dut.io.pads.gpio(8).i.ival  := 0.U

    // Header row 3: A0-A5 (we don't support using them as analog inputs)
    // just treat them as regular digital GPIOs
    IOBUF(ck_io(15), dut.io.pads.gpio(9))  // A1 = CS(2)
    IOBUF(ck_io(16), dut.io.pads.gpio(10)) // A2 = CS(3) / PWM2(0)
    IOBUF(ck_io(17), dut.io.pads.gpio(11)) // A3 = PWM2(1)
    IOBUF(ck_io(18), dut.io.pads.gpio(12)) // A4 = PWM2(2) / SDA
    IOBUF(ck_io(19), dut.io.pads.gpio(13)) // A5 = PWM2(3) / SCL

    // Mirror outputs of GPIOs with PWM peripherals to RGB LEDs on Arty
    // assign RGB LED0 R,G,B inputs = PWM0(1,2,3) when iof_1 is active
    led0_r := dut.io.pads.gpio(1).o.oval & dut.io.pads.gpio(1).o.oe
    led0_g := dut.io.pads.gpio(2).o.oval & dut.io.pads.gpio(2).o.oe
    led0_b := dut.io.pads.gpio(3).o.oval & dut.io.pads.gpio(3).o.oe

    // Note that this is the one which is actually connected on the HiFive/Crazy88
    // Board. Same with RGB LED1 R,G,B inputs = PWM1(1,2,3) when iof_1 is active
    led1_r := dut.io.pads.gpio(19).o.oval & dut.io.pads.gpio(19).o.oe
    led1_g := dut.io.pads.gpio(21).o.oval & dut.io.pads.gpio(21).o.oe
    led1_b := dut.io.pads.gpio(22).o.oval & dut.io.pads.gpio(22).o.oe

    // and RGB LED2 R,G,B inputs = PWM2(1,2,3) when iof_1 is active
    led2_r := dut.io.pads.gpio(11).o.oval & dut.io.pads.gpio(11).o.oe
    led2_g := dut.io.pads.gpio(12).o.oval & dut.io.pads.gpio(12).o.oe
    led2_b := dut.io.pads.gpio(13).o.oval & dut.io.pads.gpio(13).o.oe

    // Only 19 out of 20 shield pins connected to GPIO pads
    // Shield pin A5 (pin 14) left unconnected
    // The buttons are connected to some extra GPIO pads not connected on the
    // HiFive1 board
    IOBUF(btn_0, dut.io.pads.gpio(15))
    IOBUF(btn_1, dut.io.pads.gpio(30))
    IOBUF(btn_2, dut.io.pads.gpio(31))

    val iobuf_btn_3 = Module(new IOBUF())
    iobuf_btn_3.io.I := ~dut.io.pads.aon.pmu.dwakeup_n.o.oval
    iobuf_btn_3.io.T := ~dut.io.pads.aon.pmu.dwakeup_n.o.oe
    attach(btn_3, iobuf_btn_3.io.IO)
    dut.io.pads.aon.pmu.dwakeup_n.i.ival := ~iobuf_btn_3.io.O & dut.io.pads.aon.pmu.dwakeup_n.o.ie

    // UART1 RX/TX pins are assigned to PMOD_D connector pins 0/1
    IOBUF(ja_0, dut.io.pads.gpio(25)) // UART1 TX
    IOBUF(ja_1, dut.io.pads.gpio(24)) // UART1 RX

    // SPI2 pins mapped to 6 pin ICSP connector (standard on later
    // arduinos) These are connected to some extra GPIO pads not connected
    // on the HiFive1 board
    IOBUF(ck_ss,   dut.io.pads.gpio(26))
    IOBUF(ck_mosi, dut.io.pads.gpio(27))
    IOBUF(ck_miso, dut.io.pads.gpio(28))
    IOBUF(ck_sck,  dut.io.pads.gpio(29))

    // Use the LEDs for some more useful debugging things
    led_0 := ck_rst
    led_1 := SRST_n
    led_2 := dut.io.pads.aon.pmu.dwakeup_n.i.ival
    IOBUF(led_3, dut.io.pads.gpio(14))

    //---------------------------------------------------------------------
    // Unconnected inputs
    //---------------------------------------------------------------------

    dut.io.pads.aon.erst_n.i.ival       := ~reset_periph
    dut.io.pads.aon.lfextclk.i.ival     := slow_clock
    dut.io.pads.aon.pmu.vddpaden.i.ival := 1.U
  }
}
