package gemmini

import chisel3._

object GemminiISA {
  //==========================================================================
  // original gemmini opcodes
  //==========================================================================
  val CONFIG_CMD           = 0.U(7.W) 
  val LOAD_CMD             = 2.U(7.W)
  val STORE_CMD            = 3.U(7.W)
  val COMPUTE_AND_FLIP_CMD = 4.U(7.W)
  val COMPUTE_AND_STAY_CMD = 5.U(7.W)
  val PRELOAD_CMD          = 6.U(7.W)
  val FLUSH_CMD            = 7.U(7.W)
  val LOOP_WS              = 8.U(7.W)

  //==========================================================================
  // New Gemmini2 opcodes
  //==========================================================================
  val COMPUTE_ALL =  4.U(7.W) // same as COMPUTE_AND_FLIP
  val ADDR_AB     = 10.U(7.W)
  val ADDR_CD     = 11.U(7.W)
  val SIZE0       = 12.U(7.W)
  val SIZE1       = 13.U(7.W)
  val RPT_BIAS    = 14.U(7.W)
  val RESET       = 15.U(7.W)

  //==========================================================================
  // config-opcode parameters (rs1[1:0] values))))
  //==========================================================================
  val CONFIG_EX     = 0.U(2.W)
  val CONFIG_LOAD   = 1.U(2.W)
  val CONFIG_STORE  = 2.U(2.W)

  //==========================================================================
  // dataflow configuration
  //==========================================================================
  val GARBAGE_ADDR      = "hffffffff".U(32.W)
  val OUTPUT_STATIONARY =  0.U(1.W)
  val WEIGHT_STATIONARY =  1.U(1.W)

  //==========================================================================
  // activation opcodes
  //==========================================================================
  val NO_ACTIVATION = 0.U(2.W)
  val RELU          = 1.U(2.W)
  val RELU6         = 2.U(2.W)
}

