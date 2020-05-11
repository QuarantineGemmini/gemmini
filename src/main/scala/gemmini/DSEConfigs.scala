package gemmini

import chisel3._
import freechips.rocketchip.config.{Config, Parameters}
import freechips.rocketchip.diplomacy.{LazyModule, ValName}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tile.{BuildRoCC, OpcodeSet}

//============================================================================
// Design Space Exploration Settings
//============================================================================
object DSEBaseConfig {
  val baseConfig = GemminiArrayConfig(
    tileRows        = 1,
    tileColumns     = 1,
    meshRows        = 16,
    meshColumns     = 16,
    ld_queue_length = 4,
    st_queue_length = 2,
    ex_queue_length = 8,
    rob_entries     = 8,
    sp_banks        = 4, // TODO support one-bank designs
    acc_banks       = 1,
    sp_capacity     = CapacityInKilobytes(64),
    shifter_banks   = 1, // TODO add separate parameters for left and up shifter banks
    dataflow        = Dataflow.OS,
    acc_capacity    = CapacityInKilobytes(16),
    mem_pipeline    = 1,
    dma_maxbytes    = 128, // TODO get this from cacheblockbytes
    dma_buswidth    = 128, // TODO get this from SystemBusKey
    aligned_to      = 16,
    inputType       = SInt(8.W),
    outputType      = SInt(19.W),
    accType         = SInt(32.W),
  )
}

object DSEConfigs {
  import DSEBaseConfig.{baseConfig => base}
  val baseConfig = base.copy(
    headerFileName = "gemmini_params_dse1.h"
  )
  val wsOnlyConfig = baseConfig.copy(
    dataflow = Dataflow.WS,
    headerFileName = "gemmini_params_dse2.h"
  )
  val bothDataflowsConfig = baseConfig.copy(
    dataflow = Dataflow.BOTH,
    headerFileName = "gemmini_params_dse3.h"
  )
  val highBitwidthConfig = baseConfig.copy(
    inputType = SInt(32.W),
    outputType = SInt(32.W),
    headerFileName = "gemmini_params_dse4.h"
  )
  val largerDimConfig = baseConfig.copy(
    meshRows = 32,
    meshColumns = 32,
    outputType = SInt(20.W),
    headerFileName = "gemmini_params_dse5.h"
  )
  val fullyCombinationalConfig = baseConfig.copy(
    tileRows = 16,
    tileColumns = 16,
    meshRows = 1,
    meshColumns = 1,
    headerFileName = "gemmini_params_dse6.h"
  )
  val moreMemoryConfig = baseConfig.copy(
    sp_capacity = CapacityInKilobytes(256),
    acc_capacity = CapacityInKilobytes(64),
    headerFileName = "gemmini_params_dse7.h"
  )
  val moreBanksConfig = baseConfig.copy(
    sp_banks = 32,
    headerFileName = "gemmini_params_dse8.h"
  )
  val narrowerBusConfig = baseConfig.copy(
    dma_maxbytes = 64,
    dma_buswidth = 64,
    headerFileName = "gemmini_params_dse10.h"
  )
  val pnr16Config = baseConfig.copy(
    sp_capacity = CapacityInKilobytes(256),
    acc_capacity = CapacityInKilobytes(64),
    dataflow = Dataflow.BOTH,
    headerFileName = "gemmini_params_pnr16.h"
  )
  val pnr32Config = baseConfig.copy(
    sp_capacity = CapacityInKilobytes(512),
    acc_capacity = CapacityInKilobytes(128),
    meshRows = 32,
    meshColumns = 32,
    outputType = SInt(20.W),
    dataflow = Dataflow.BOTH,
    headerFileName = "gemmini_params_pnr32.h"
  )
}

//============================================================================
// Design Space Exploration Rocket-Chip Mixins
//============================================================================

// BASELINE
object WithDSE1GemminiConfig {
  def apply() = WithGemminiConfig(DSEConfigs.baseConfig)
}

// DATAFLOW CHANGE: WS
object WithDSE2GemminiConfig {
  def apply() = WithGemminiConfig(DSEConfigs.wsOnlyConfig)
}

// DATAFLOW CHANGE: BOTH
object WithDSE3GemminiConfig {
  def apply() = WithGemminiConfig(DSEConfigs.bothDataflowsConfig)
}

// BITWIDTH CHANGE: 32 BITS
object WithDSE4GemminiConfig {
  def apply() = WithGemminiConfig(DSEConfigs.highBitwidthConfig)
}

// DIMENSIONS CHANGE: 32x32
object WithDSE5GemminiConfig {
  def apply() = WithGemminiConfig(DSEConfigs.largerDimConfig)
}

// PIPELINE DEPTH CHANGE: Fully Combinational
object WithDSE6GemminiConfig {
  def apply() = WithGemminiConfig(DSEConfigs.fullyCombinationalConfig)
}

// MEMORY CAPACITY CHANGE: 256 KB
object WithDSE7GemminiConfig {
  def apply() = WithGemminiConfig(DSEConfigs.moreMemoryConfig)
}

// MEMORY BANKS CHANGE: 33 Banks
object WithDSE8GemminiConfig {
  def apply() = WithGemminiConfig(DSEConfigs.moreBanksConfig)
}

// BUS WIDTH CHANGE: 64 bits
object WithDSE10GemminiConfig {
  def apply() = WithGemminiConfig(DSEConfigs.narrowerBusConfig)
}

// PnR 16-by-16
object WithPnR16GemminiConfig {
  def apply() = WithGemminiConfig(DSEConfigs.pnr16Config)
}

// PnR 32-by-32
object WithPnR32GemminiConfig {
  def apply() = WithGemminiConfig(DSEConfigs.pnr32Config)
}

//============================================================================
// Design Space Exploration Top Level Configs
//============================================================================
class GemminiDSE1Config extends Config(
  WithDSE1GemminiConfig() ++
  new freechips.rocketchip.system.DefaultConfig
)
class GemminiDSE2Config extends Config(
  WithDSE2GemminiConfig() ++
  new freechips.rocketchip.system.DefaultConfig
)
class GemminiDSE3Config extends Config(
  WithDSE3GemminiConfig() ++
  new freechips.rocketchip.system.DefaultConfig
)
class GemminiDSE4Config extends Config(
  WithDSE4GemminiConfig() ++
  new freechips.rocketchip.system.DefaultConfig
)
class GemminiDSE5Config extends Config(
  WithDSE5GemminiConfig() ++
  new freechips.rocketchip.system.DefaultConfig
)
class GemminiDSE6Config extends Config(
  WithDSE6GemminiConfig() ++
  new freechips.rocketchip.system.DefaultConfig
)
class GemminiDSE7Config extends Config(
  WithDSE7GemminiConfig() ++
  new freechips.rocketchip.system.DefaultConfig
)
class GemminiDSE8Config extends Config(
  WithDSE8GemminiConfig() ++
  new freechips.rocketchip.system.DefaultConfig
)
class GemminiDSE10Config extends Config(
  WithDSE10GemminiConfig() ++
  new freechips.rocketchip.system.DefaultConfig
)
class GemminiPnr16Config extends Config(
  WithPnR16GemminiConfig() ++
  new freechips.rocketchip.system.DefaultConfig
)
class GemminiPnr32Config extends Config(
  WithPnR32GemminiConfig() ++
  new freechips.rocketchip.system.DefaultConfig
)
