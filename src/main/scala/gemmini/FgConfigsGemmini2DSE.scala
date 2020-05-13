package gemmini

import chisel3._
import freechips.rocketchip.config.{Config, Parameters}
import freechips.rocketchip.diplomacy.{LazyModule, ValName}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tile.{BuildRoCC, OpcodeSet}

//============================================================================
// Design Space Exploration Settings
//============================================================================
object FgGemmini2DSEBaseConfig {
  val baseConfig = FgGemminiArrayConfig(
    dim                 = 32,
    fg_dim              = 8,
    //--------------------
    mem_op_queue_length = 8,
    ex_queue_length     = 8,
    rob_entries         = 16,
    //--------------------
    dma_max_req_bytes   = 64, // TODO: how do i change CacheBlockBytes?
    dma_beat_bits       = 128,
    //--------------------
    inputType           = SInt(8.W),
    outputType          = SInt(24.W),
    accType             = SInt(32.W),
  )
}

object FgGemmini2DSEConfigs {
  import FgGemmini2DSEBaseConfig.{baseConfig => base}
  val dim32fg32 = base.copy(dim = 32, fg_dim = 32)
  val dim32fg16 = base.copy(dim = 32, fg_dim = 16)
  val dim32fg8  = base.copy(dim = 32, fg_dim = 8)
  val dim16fg16 = base.copy(dim = 16, fg_dim = 16)
  val dim16fg8  = base.copy(dim = 16, fg_dim = 8)
  val dim16fg4  = base.copy(dim = 16, fg_dim = 4)
}

//============================================================================
// Design Space Exploration Rocket-Chip Mixins (gemmini2)
//============================================================================

object WithFgGemmini2dim32fg32 {
  def apply() = WithFgGemmini2Config(FgGemmini2DSEConfigs.dim32fg32)
}
object WithFgGemmini2dim32fg16 {
  def apply() = WithFgGemmini2Config(FgGemmini2DSEConfigs.dim32fg16)
}
object WithFgGemmini2dim32fg8 {
  def apply() = WithFgGemmini2Config(FgGemmini2DSEConfigs.dim32fg8)
}
object WithFgGemmini2dim16fg16 {
  def apply() = WithFgGemmini2Config(FgGemmini2DSEConfigs.dim16fg16)
}
object WithFgGemmini2dim16fg8 {
  def apply() = WithFgGemmini2Config(FgGemmini2DSEConfigs.dim16fg8)
}
object WithFgGemmini2dim16fg4 {
  def apply() = WithFgGemmini2Config(FgGemmini2DSEConfigs.dim16fg4)
}

