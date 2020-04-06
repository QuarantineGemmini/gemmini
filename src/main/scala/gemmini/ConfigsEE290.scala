package gemmini

import chisel3._
import freechips.rocketchip.config.{Config, Parameters}
import freechips.rocketchip.diplomacy.{LazyModule, ValName}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tile.{BuildRoCC, OpcodeSet}

//============================================================================
// EE290 Class Config Mixins
//============================================================================
object GemminiEE290Configs{
  import GemminiConfigs.{defaultConfig => base}
  val defaultConfig = base.copy()

  val Lab2Config = defaultConfig.copy(
    MESH_ROWS    = 8,
    MESH_COLS    = 8,
    DATAFLOW     = Dataflow.WS
  )

  val Lab2LargeSPConfig = defaultConfig.copy(
    MESH_ROWS    = 8,
    MESH_COLS    = 8,
    DATAFLOW     = Dataflow.WS,
    SP_CAPACITY  = CapacityInKilobytes(2048)
  )

  val Lab3Config = defaultConfig.copy(
    MESH_ROWS    = 32,
    MESH_COLS    = 32,
    DATAFLOW     = Dataflow.WS,
    OUTPUT_TYPE  = SInt(21.W)
  )

  val Lab3SmallSPConfig = defaultConfig.copy(
    MESH_ROWS    = 32,
    MESH_COLS    = 32,
    DATAFLOW     = Dataflow.WS,
    SP_CAPACITY  = CapacityInKilobytes(128),
    ACC_CAPACITY = CapacityInKilobytes(32),
    OUTPUT_TYPE  = SInt(21.W)
  )
}

//============================================================================
// EE290 Configs
//============================================================================
object WithEE290Lab2GemminiConfig {
  def apply = WithGemminiConfig(GemminiEE290Configs.Lab2Config)
}
object WithEE290Lab2LargeSPGemminiConfig {
  def apply = WithGemminiConfig(GemminiEE290Configs.Lab2LargeSPConfig)
}
object WithEE290Lab3GemminiConfig {
  def apply = WithGemminiConfig(GemminiEE290Configs.Lab3Config)
}
object WithEE290Lab3SmallSPGemminiConfig {
  def apply = WithGemminiConfig(GemminiEE290Configs.Lab3SmallSPConfig)
}
