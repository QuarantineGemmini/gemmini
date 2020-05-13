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
  val ADDR_AB_CMD   = 10.U(7.W)
  val ADDR_CD_CMD   = 11.U(7.W)
  val SIZE0_CMD     = 12.U(7.W)
  val SIZE1_CMD     = 13.U(7.W)
  val RPT_BIAS_CMD  = 14.U(7.W)
  val RESET_CMD     = 15.U(7.W)
  val COMPUTE_CMD   = 16.U(7.W)

  // opcodes for im2col addition
  val CONFIG_ADDR_MODE_A_CMD = 17.U(7.W) 
  val CONFIG_ADDR_MODE_B_CMD = 18.U(7.W) 
  val CONFIG_ADDR_MODE_C_CMD = 19.U(7.W) 
  val CONFIG_ADDR_MODE_D_CMD = 20.U(7.W) 

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
  // addressing mode for A,B,C,D operands
  //==========================================================================
  val ADDR_MODE_NORMAL = 0.U(1.W)
  val ADDR_MODE_IM2COL = 1.U(1.W)

  //==========================================================================
  // activation opcodes
  //==========================================================================
  val NO_ACTIVATION = 0.U(2.W)
  val RELU          = 1.U(2.W)
  val RELU6         = 2.U(2.W)
}

object Dataflow extends Enumeration {
  val OS, WS, BOTH = Value
}

object AddressMode extends Enumeration {
  val NORMAL, IM2COL = Value
}

