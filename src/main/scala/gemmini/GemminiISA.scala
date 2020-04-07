package gemmini

import chisel3._

object GemminiISA {
  //==========================================================================
  // original gemmini opcodes
  //==========================================================================
  val CONFIG_CMD           = 0.U
  val LOAD_CMD             = 2.U
  val STORE_CMD            = 3.U
  val COMPUTE_AND_FLIP_CMD = 4.U
  val COMPUTE_AND_STAY_CMD = 5.U
  val PRELOAD_CMD          = 6.U
  val FLUSH_CMD            = 7.U
  val LOOP_WS              = 8.U

  //==========================================================================
  // New Gemmini2 opcodes
  //==========================================================================
  val COMPUTE_ALL = 4.U  // same as COMPUTE_AND_FLIP
  val ADDR_AB     = 10.U
  val ADDR_CD     = 11.U
  val SIZE0       = 12.U
  val SIZE1       = 13.U
  val RPT_BIAS    = 14.U
  val RESET       = 15.U

  //==========================================================================
  // config-opcode parameters (rs1[1:0] values))))
  //==========================================================================
  val CONFIG_LOAD   = 1.U(2)
  val CONFIG_STORE  = 2.U(2)
  val CONFIG_EX     = 0.U(2)

  //==========================================================================
  // dataflow configuration
  //==========================================================================
  val GARBAGE_ADDR      = 0xffffffff.U(32)
  val OUTPUT_STATIONARY =  0.U(1)
  val WEIGHT_STATIONARY =  1.U(1)

  //==========================================================================
  // activation opcodes
  //==========================================================================
  val NO_ACTIVATION = 0.U(2)
  val RELU          = 1.U(2)
  val RELU6         = 2.U(2)
}

