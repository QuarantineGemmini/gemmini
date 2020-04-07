package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._
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
    meshRows     = 8,
    meshColumns  = 8,
    dataflow     = Dataflow.WS
  )

  val Lab2LargeSPConfig = defaultConfig.copy(
    meshRows     = 8,
    meshColumns  = 8,
    dataflow     = Dataflow.WS,
    sp_capacity  = CapacityInKilobytes(2048)
  )

  print("got here\n")
  val Lab3Config = defaultConfig.copy(
    meshRows     = 32,
    meshColumns  = 32,
    dataflow     = Dataflow.WS,
    outputType   = SInt(21.W)
  )

  val Lab3SmallSPConfig = defaultConfig.copy(
    meshRows     = 32,
    meshColumns  = 32,
    dataflow     = Dataflow.WS,
    sp_capacity  = CapacityInKilobytes(128),
    acc_capacity = CapacityInKilobytes(32),
    outputType   = SInt(21.W)
  )
}

//============================================================================
// EE290 Configs
//============================================================================
object WithEE290Lab2GemminiConfig {
  def apply(dummy:Int=0) 
    = WithGemminiConfig(GemminiEE290Configs.Lab2Config)
}
object WithEE290Lab2LargeSPGemminiConfig {
  def apply(dummy:Int=0) 
    = WithGemminiConfig(GemminiEE290Configs.Lab2LargeSPConfig)
}
object WithEE290Lab3GemminiConfig {
  def apply(dummy:Int=0) 
    = WithGemminiConfig(GemminiEE290Configs.Lab3Config)
}
object WithEE290Lab3SmallSPGemminiConfig {
  def apply(dummy:Int=0) 
    = WithGemminiConfig(GemminiEE290Configs.Lab3SmallSPConfig)
}

//============================================================================
// EE290 Configs (gemmini2)
//============================================================================
object WithEE290Lab3Gemmini2Config {
  def apply(dummy:Int=0) 
    = WithGemminiConfig(GemminiEE290Configs.Lab3Config)
}
