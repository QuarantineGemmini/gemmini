package gemmini

import chisel3._
import freechips.rocketchip.config.{Config, Parameters}
import freechips.rocketchip.diplomacy.{LazyModule, ValName}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tile.{BuildRoCC, OpcodeSet}

//============================================================================
// Design Space Exploration Settings
//============================================================================
object Gemmini2DSEBaseConfig {
  val baseConfig = GemminiArrayConfig(
    tileRows        = 1,
    tileColumns     = 1,
    meshRows        = 16,
    meshColumns     = 16,
    ld_queue_length = 8,
    st_queue_length = 4,
    ex_queue_length = 8,
    rob_entries     = 16,
    sp_banks        = 4, // TODO support one-bank designs
    acc_banks       = 1,
    sp_capacity     = CapacityInKilobytes(64),
    shifter_banks   = 1, // TODO add parameters for left/up shifter banks
    dataflow        = Dataflow.WS,
    acc_capacity    = CapacityInKilobytes(16),
    mem_pipeline    = 1,
    dma_maxbytes    = 64, // ssteffl TODO: how do i change CacheBhockBytes???
    dma_buswidth    = 128,
    aligned_to      = 1,
    inputType       = SInt(8.W),
    outputType      = SInt(19.W),
    accType         = SInt(32.W),
  )
}

object Gemmini2DSEConfigs {
  import Gemmini2DSEBaseConfig.{baseConfig => base}
  //-------------------------------------------------------------------------
  val dim16sp64acc16bus064 = base.copy(
    dma_buswidth   = 64,
    headerFileName = "gemmini_params_dim16sp64acc16bus064.h"
  )
  val dim32sp64acc16bus064 = base.copy(
    meshRows       = 32,
    meshColumns    = 32,
    dma_buswidth   = 64,
    headerFileName = "gemmini_params_dim32sp64acc16bus064.h"
  )
  val dim16sp16acc64bus064 = base.copy(
    sp_capacity    = CapacityInKilobytes(32),
    acc_capacity   = CapacityInKilobytes(64),
    dma_buswidth   = 64,
    headerFileName = "gemmini_params_dim16sp16acc64bus064.h"
  )
  val dim32sp16acc64bus064 = base.copy(
    meshRows       = 32,
    meshColumns    = 32,
    sp_capacity    = CapacityInKilobytes(32),
    acc_capacity   = CapacityInKilobytes(64),
    dma_buswidth   = 64,
    headerFileName = "gemmini_params_dim32sp16acc64bus064.h"
  )
  //-------------------------------------------------------------------------
  val dim16sp64acc16bus128 = base.copy(
    headerFileName = "gemmini_params_dim16sp64acc16bus128.h"
  )
  val dim32sp64acc16bus128 = base.copy(
    meshRows       = 32,
    meshColumns    = 32,
    headerFileName = "gemmini_params_dim32sp64acc16bus128.h"
  )
  val dim16sp16acc64bus128 = base.copy(
    sp_capacity    = CapacityInKilobytes(32),
    acc_capacity   = CapacityInKilobytes(64),
    headerFileName = "gemmini_params_dim16sp16acc64bus128.h"
  )
  val dim32sp16acc64bus128 = base.copy(
    // TODO TODO TODO: actually look into the dma_buswidth...
    //dma_buswidth   = 256,
    meshRows       = 32,
    meshColumns    = 32,
    sp_capacity    = CapacityInKilobytes(32),
    acc_capacity   = CapacityInKilobytes(64),
    headerFileName = "gemmini_params_dim32sp16acc64bus128.h"
  )
}

//============================================================================
// Design Space Exploration Rocket-Chip Mixins (gemmini2)
//============================================================================

object WithGemmini2dim16sp64acc16bus064 {
  def apply() = WithGemmini2Config(Gemmini2DSEConfigs.dim16sp64acc16bus064)
}
object WithGemmini2dim32sp64acc16bus064 {
  def apply() = WithGemmini2Config(Gemmini2DSEConfigs.dim32sp64acc16bus064)
}
object WithGemmini2dim16sp16acc64bus064 {
  def apply() = WithGemmini2Config(Gemmini2DSEConfigs.dim16sp16acc64bus064)
}
object WithGemmini2dim32sp16acc64bus064 {
  def apply() = WithGemmini2Config(Gemmini2DSEConfigs.dim32sp16acc64bus064)
}
object WithGemmini2dim16sp64acc16bus128 {
  def apply() = WithGemmini2Config(Gemmini2DSEConfigs.dim16sp64acc16bus128)
}
object WithGemmini2dim32sp64acc16bus128 {
  def apply() = WithGemmini2Config(Gemmini2DSEConfigs.dim32sp64acc16bus128)
}
object WithGemmini2dim16sp16acc64bus128 {
  def apply() = WithGemmini2Config(Gemmini2DSEConfigs.dim16sp16acc64bus128)
}
object WithGemmini2dim32sp16acc64bus128 {
  def apply() = WithGemmini2Config(Gemmini2DSEConfigs.dim32sp16acc64bus128)
}

//============================================================================
// Design Space Exploration Rocket-Chip Mixins (gemmini1 reference!!!)
// - notice the WithGemminiConfig instead of WithGemmini2Config
//============================================================================

object WithGemminidim16sp64acc16bus064 {
  def apply() = WithGemminiConfig(Gemmini2DSEConfigs.dim16sp64acc16bus064)
}
object WithGemminidim32sp64acc16bus064 {
  def apply() = WithGemminiConfig(Gemmini2DSEConfigs.dim32sp64acc16bus064)
}
object WithGemminidim16sp16acc64bus064 {
  def apply() = WithGemminiConfig(Gemmini2DSEConfigs.dim16sp16acc64bus064)
}
object WithGemminidim32sp16acc64bus064 {
  def apply() = WithGemminiConfig(Gemmini2DSEConfigs.dim32sp16acc64bus064)
}
object WithGemminidim16sp64acc16bus128 {
  def apply() = WithGemminiConfig(Gemmini2DSEConfigs.dim16sp64acc16bus128)
}
object WithGemminidim32sp64acc16bus128 {
  def apply() = WithGemminiConfig(Gemmini2DSEConfigs.dim32sp64acc16bus128)
}
object WithGemminidim16sp16acc64bus128 {
  def apply() = WithGemminiConfig(Gemmini2DSEConfigs.dim16sp16acc64bus128)
}
object WithGemminidim32sp16acc64bus128 {
  def apply() = WithGemminiConfig(Gemmini2DSEConfigs.dim32sp16acc64bus128)
}

