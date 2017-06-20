// See LICENSE for license details.
package sifive.freedom.everywhere.e300artydevkit

import Chisel._
import config._
import diplomacy._
import coreplex._
import rocketchip._

import sifive.blocks.util.{ResetCatchAndSync}
import sifive.blocks.devices.mockaon._
import sifive.blocks.devices.gpio._
import sifive.blocks.devices.pwm._
import sifive.blocks.devices.spi._
import sifive.blocks.devices.uart._
import sifive.blocks.devices.i2c._

//-------------------------------------------------------------------------
// E300ArtyDevKitCoreplex
//-------------------------------------------------------------------------

class E300ArtyDevKitCoreplex(implicit p: Parameters) extends BareCoreplex
    with CoreplexNetwork
    with CoreplexRISCVPlatform
    with HasRocketTiles {
  override lazy val module = new E300ArtyDevKitCoreplexModule(this, () =>
new E300ArtyDevKitCoreplexBundle(this))
}

class E300ArtyDevKitCoreplexBundle[+L <: E300ArtyDevKitCoreplex](_outer: L)
extends BareCoreplexBundle(_outer)
    with CoreplexNetworkBundle
    with CoreplexRISCVPlatformBundle
    with HasRocketTilesBundle

class E300ArtyDevKitCoreplexModule[+L <: E300ArtyDevKitCoreplex, +B <:
E300ArtyDevKitCoreplexBundle[L]](_outer: L, _io: () => B)
  extends BareCoreplexModule(_outer, _io)
    with CoreplexNetworkModule
    with CoreplexRISCVPlatformModule
    with HasRocketTilesModule

//-------------------------------------------------------------------------
// E300ArtyDevKitSystem
//-------------------------------------------------------------------------

class E300ArtyDevKitSystem(implicit p: Parameters) extends BaseSystem
    with HasPeripheryBootROM
    with HasPeripheryDebug
    with HasPeripheryMockAON
    with HasPeripheryUART
    with HasPeripherySPIFlash
    with HasPeripherySPI
    with HasPeripheryGPIO
    with HasPeripheryPWM
    with HasPeripheryI2C {
  override lazy val module = new E300ArtyDevKitSystemModule(this)

  val coreplex = LazyModule(new E300ArtyDevKitCoreplex)
  socBus.node := coreplex.mmio
  coreplex.mmioInt := intBus.intnode
}

class E300ArtyDevKitSystemModule[+L <: E300ArtyDevKitSystem](_outer: L)
  extends BaseSystemModule(_outer)
    with HasPeripheryBootROMModuleImp
    with HasPeripheryDebugModuleImp
    with HasPeripheryUARTModuleImp
    with HasPeripherySPIModuleImp
    with HasPeripheryGPIOModuleImp
    with HasPeripherySPIFlashModuleImp
    with HasPeripheryMockAONModuleImp
    with HasPeripheryPWMModuleImp
    with HasPeripheryI2CModuleImp

//-------------------------------------------------------------------------
// E300ArtyDevKitTopIO
//-------------------------------------------------------------------------

class E300ArtyDevKitTopIO(implicit val p: Parameters) extends Bundle {
  val pads = new Bundle {
    val jtag = new JTAGPinsIO
    val gpio = Vec(p(PeripheryGPIOKey)(0).width, new GPIOPin)
    val qspi = new SPIPinsIO(p(PeripherySPIFlashKey)(0))
    val aon = new MockAONWrapperPadsIO()
  }
  val jtag_reset = Bool(INPUT)
}

//-------------------------------------------------------------------------
// E300ArtyDevKitTop
//-------------------------------------------------------------------------

class E300ArtyDevKitTop(implicit val p: Parameters) extends Module {
  val sys = Module(LazyModule(new E300ArtyDevKitSystem).module)
  val io = new E300ArtyDevKitTopIO

  // This needs to be de-asserted synchronously to the coreClk.
  val async_corerst = sys.aon.rsts.corerst
  // Add in debug-controlled reset.
  sys.reset := ResetCatchAndSync(clock, async_corerst, 20)

  //-----------------------------------------------------------------------
  // Check for unsupported rocket-chip connections
  //-----------------------------------------------------------------------

  require (p(NExtTopInterrupts) == 0, "No Top-level interrupts supported");

  //-----------------------------------------------------------------------
  // Build GPIO Pin Mux
  //-----------------------------------------------------------------------
  // Pin Mux for UART, SPI, PWM
  // First convert the System outputs into "IOF" using the respective *GPIOPort
  // converters.

  val sys_uarts = sys.uarts
  val sys_pwms  = sys.pwms
  val sys_spis  = sys.spis
  val sys_i2cs  = sys.i2cs

  val uart_pins = sys.outer.uartParams.map { c => Module (new UARTGPIOPort) }
  val pwm_pins  = sys.outer.pwmParams.map  { c => Module (new PWMGPIOPort(c)) }
  val spi_pins  = sys.outer.spiParams.map  { c => Module (new SPIGPIOPort(c)) }
  val i2c_pins  = sys.outer.i2cParams.map  { c => Module (new I2CGPIOPort) }

  (uart_pins zip sys_uarts) map {case (p, r) => p.io.uart <> r}
  (pwm_pins zip  sys_pwms)  map {case (p, r) => p.io.pwm  <> r}
  (spi_pins zip  sys_spis)  map {case (p, r) => p.io.spi  <> r}
  (i2c_pins zip  sys_i2cs)  map {case (p, r) => p.io.i2c  <> r}

  //-----------------------------------------------------------------------
  // Default Pin connections before attaching pinmux

  for (iof_0 <- sys.gpio(0).iof_0.get) {
    iof_0.o := GPIOPinIOFCtrl()
  }

  for (iof_1 <- sys.gpio(0).iof_1.get) {
    iof_1.o := GPIOPinIOFCtrl()
  }

  //-----------------------------------------------------------------------
  // TODO: Make this mapping more programmatic.

  val iof_0 = sys.gpio(0).iof_0.get
  val iof_1 = sys.gpio(0).iof_1.get

  // SPI1 (0 is the dedicated)
  GPIOPinToIOF(spi_pins(0).io.pins.cs(0), iof_0(2))
  GPIOPinToIOF(spi_pins(0).io.pins.dq(0), iof_0(3))
  GPIOPinToIOF(spi_pins(0).io.pins.dq(1), iof_0(4))
  GPIOPinToIOF(spi_pins(0).io.pins.sck,   iof_0(5))
  GPIOPinToIOF(spi_pins(0).io.pins.dq(2), iof_0(6))
  GPIOPinToIOF(spi_pins(0).io.pins.dq(3), iof_0(7))
  GPIOPinToIOF(spi_pins(0).io.pins.cs(1), iof_0(8))
  GPIOPinToIOF(spi_pins(0).io.pins.cs(2), iof_0(9))
  GPIOPinToIOF(spi_pins(0).io.pins.cs(3), iof_0(10))

  // SPI2
  GPIOPinToIOF(spi_pins(1).io.pins.cs(0), iof_0(26))
  GPIOPinToIOF(spi_pins(1).io.pins.dq(0), iof_0(27))
  GPIOPinToIOF(spi_pins(1).io.pins.dq(1), iof_0(28))
  GPIOPinToIOF(spi_pins(1).io.pins.sck,   iof_0(29))
  GPIOPinToIOF(spi_pins(1).io.pins.dq(2), iof_0(30))
  GPIOPinToIOF(spi_pins(1).io.pins.dq(3), iof_0(31))

  // I2C
  if (sys.outer.i2cParams.length == 1) {
    GPIOPinToIOF(i2c_pins(0).io.pins.sda, iof_0(12))
    GPIOPinToIOF(i2c_pins(0).io.pins.scl, iof_0(13))
  }

  // UART0
  GPIOPinToIOF(uart_pins(0).io.pins.rxd, iof_0(16))
  GPIOPinToIOF(uart_pins(0).io.pins.txd, iof_0(17))

  // UART1
  GPIOPinToIOF(uart_pins(1).io.pins.rxd, iof_0(24))
  GPIOPinToIOF(uart_pins(1).io.pins.txd, iof_0(25))

  //PWM
  GPIOPinToIOF(pwm_pins(0).io.pins.pwm(0), iof_1(0) )
  GPIOPinToIOF(pwm_pins(0).io.pins.pwm(1), iof_1(1) )
  GPIOPinToIOF(pwm_pins(0).io.pins.pwm(2), iof_1(2) )
  GPIOPinToIOF(pwm_pins(0).io.pins.pwm(3), iof_1(3) )

  GPIOPinToIOF(pwm_pins(1).io.pins.pwm(1), iof_1(19))
  GPIOPinToIOF(pwm_pins(1).io.pins.pwm(0), iof_1(20))
  GPIOPinToIOF(pwm_pins(1).io.pins.pwm(2), iof_1(21))
  GPIOPinToIOF(pwm_pins(1).io.pins.pwm(3), iof_1(22))

  GPIOPinToIOF(pwm_pins(2).io.pins.pwm(0), iof_1(10))
  GPIOPinToIOF(pwm_pins(2).io.pins.pwm(1), iof_1(11))
  GPIOPinToIOF(pwm_pins(2).io.pins.pwm(2), iof_1(12))
  GPIOPinToIOF(pwm_pins(2).io.pins.pwm(3), iof_1(13))

  //-----------------------------------------------------------------------
  // Drive actual Pads
  //-----------------------------------------------------------------------

  // Result of Pin Mux
  io.pads.gpio <> sys.gpio(0).pins

  val dedicated_spi_pins = Module (new SPIGPIOPort(sys.outer.spiFlashParams(0), syncStages=3, driveStrength=Bool(true)))
  dedicated_spi_pins.clock := sys.clock
  dedicated_spi_pins.reset := sys.reset
  io.pads.qspi <> dedicated_spi_pins.io.pins
  dedicated_spi_pins.io.spi <> sys.qspi(0)

  // JTAG Debug Interface

  val sjtag = sys.debug.systemjtag.get
  val jtag_pins = Module (new JTAGGPIOPort())
  io.pads.jtag <> jtag_pins.io.pins
  sjtag.jtag <> jtag_pins.io.jtag
  sjtag.reset := io.jtag_reset
  sjtag.mfr_id := p(JtagDTMKey).idcodeManufId.U(11.W)

  // AON Pads
  io.pads.aon <> sys.aon.pads
}
