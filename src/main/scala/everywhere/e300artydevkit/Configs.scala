// See LICENSE for license details.
package sifive.freedom.everywhere.e300artydevkit

import config._
import coreplex._
import diplomacy.{DTSModel, DTSTimebase}
import rocket._
import tile.{ResetVectorBits}
import rocketchip._

import sifive.blocks.devices.mockaon._
import sifive.blocks.devices.gpio._
import sifive.blocks.devices.pwm._
import sifive.blocks.devices.spi._
import sifive.blocks.devices.uart._
import sifive.blocks.devices.i2c._

class DefaultFreedomEConfig extends Config (
  new WithStatelessBridge        ++
  new WithNBreakpoints(2)        ++
  new WithRV32                   ++
  new TinyConfig
)

class E300ArtyDevKitConfig extends Config(
  new WithBootROMFile("./bootrom/e300artydevkit.img") ++
  new WithNExtTopInterrupts(0) ++
  new WithJtagDTM ++
  new WithL1ICacheSets(8192/32) ++
  new WithCacheBlockBytes(32) ++
  new WithL1ICacheWays(2) ++
  new WithL1DCacheSets(65536/32) ++
  new WithDefaultBtb ++
  new WithFastMulDiv ++
  new WithNMemoryChannels(0) ++
  new WithoutFPU ++
  new DefaultFreedomEConfig().alter((site,here,up) => {
    case ResetVectorBits => 0x1000
    case DTSTimebase => BigInt(32768)
    case PeripheryGPIOKey => List(
      GPIOParams(address = 0x10012000, width = 32, includeIOF = true))
    case PeripheryPWMKey => List(
      PWMParams(address = 0x10015000, cmpWidth = 8),
      PWMParams(address = 0x10025000, cmpWidth = 16),
      PWMParams(address = 0x10035000, cmpWidth = 16))
    case PeripherySPIKey => List(
      SPIParams(csWidth = 4, rAddress = 0x10024000, sampleDelay = 3),
      SPIParams(csWidth = 1, rAddress = 0x10034000, sampleDelay = 3))
    case PeripherySPIFlashKey => List(
      SPIFlashParams(
        fAddress = 0x20000000,
        rAddress = 0x10014000,
        sampleDelay = 3))
    case PeripheryUARTKey => List(
      UARTParams(address = 0x10013000),
      UARTParams(address = 0x10023000))
    case PeripheryI2CKey => List(
      I2CParams(address = 0x10016000))
    case PeripheryMockAONKey =>
      MockAONParams(address = 0x10000000)
    case JtagDTMKey => new JtagDTMConfig (
      idcodeVersion = 2,
      idcodePartNum = 0x000,
      idcodeManufId = 0x489,
      debugIdleCycles = 5)
  }
  )
)
