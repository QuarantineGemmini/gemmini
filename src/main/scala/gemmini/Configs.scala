package gemmini

import chisel3._
import freechips.rocketchip.config.{Config, Field, Parameters}
import freechips.rocketchip.diplomacy.{LazyModule, ValName}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tile.{BuildRoCC, OpcodeSet, XLen}
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import freechips.rocketchip.system._
import gemmini.Arithmetic.SIntArithmetic

//===========================================================================
// Config Generator Wrapper for gemmini (reduce boilerplate code)
//===========================================================================
object WithGemminiConfig {
  def apply[T <: Data : Arithmetic](config: GemminiArrayConfig[T]) = {
    new Config((site, here, up) => {
      case BuildRoCC => Seq((p: Parameters) => {
        implicit val q = p
        implicit val v = implicitly[ValName]
        LazyModule(new Gemmini(OpcodeSet.custom3, config))
      })
      case SystemBusKey => 
        up(SystemBusKey).copy(beatBytes = (config.dma_buswidth/8))
    })
  }
}

object WithGemmini2Config {
  def apply[T <: Data : Arithmetic](config: GemminiArrayConfig[T]) = {
    new Config((site, here, up) => {
      case BuildRoCC => Seq((p: Parameters) => {
        implicit val q = p
        implicit val v = implicitly[ValName]
        LazyModule(new Gemmini2(OpcodeSet.custom3, config))
      })
      case SystemBusKey => 
        up(SystemBusKey).copy(beatBytes = (config.dma_buswidth/8))
    })
  }
}

//===========================================================================
// Default Gemmini Configuration
//===========================================================================
object GemminiConfigs {
  val defaultConfig = GemminiArrayConfig(
    tileRows        = 1,
    tileColumns     = 1,
    meshRows        = 8,
    meshColumns     = 8,
    ld_queue_length = 8,
    st_queue_length = 2,
    ex_queue_length = 8,
    rob_entries     = 16,
    sp_banks        = 4,
    acc_banks       = 1,
    sp_capacity     = CapacityInKilobytes(256),
    shifter_banks   = 1, // TODO add parameters for left/up shifter banks
    dataflow        = Dataflow.WS,
    acc_capacity    = CapacityInKilobytes(64),
    mem_pipeline    = 1,
    dma_maxbytes    = 128, // TODO get this from cacheblockbytes
    dma_buswidth    = 128, // TODO get this from SystemBusKey
    aligned_to      = 1,
    inputType       = SInt(8.W),
    outputType      = SInt(19.W),
    accType         = SInt(32.W),
    pe_latency      = 0
  )
}

object WithDefaultGemminiConfig {
  def apply(dummy:Int=0) 
    = WithGemminiConfig(GemminiConfigs.defaultConfig)
}

object WithDefaultGemmini2Config {
  def apply(dummy:Int=0) 
    = WithGemmini2Config(GemminiConfigs.defaultConfig)
}

//===========================================================================
// Top Level Configs
//===========================================================================
class GemminiConfig extends Config(
  WithDefaultGemminiConfig() ++
  new freechips.rocketchip.system.DefaultConfig
)

class Gemmini2Config extends Config(
  WithDefaultGemmini2Config() ++
  new freechips.rocketchip.system.DefaultConfig
)

