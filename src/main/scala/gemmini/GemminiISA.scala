package gemmini

import chisel3._

object GemminiISA {
  //==========================================================================
  // original gemmini opcodes
  //==========================================================================
  val RESET_CMD            = 0.U(7.W)
  val FLUSH_CMD            = 1.U(7.W)
  val CONFIG_DATAPATH_CMD  = 2.U(7.W) 
  val CONFIG_ADDR_CMD      = 3.U(7.W) 
  val ADDR_AB_CMD          = 4.U(7.W)
  val ADDR_CD_CMD          = 5.U(7.W)

  val LOAD_CMD             = 10.U(7.W)
  val STORE_CMD            = 11.U(7.W)
  val COMPUTE_AND_FLIP_CMD = 12.U(7.W)
  val COMPUTE_AND_STAY_CMD = 13.U(7.W)
  val PRELOAD_CMD          = 14.U(7.W)
  val LOOP_WS              = 15.U(7.W)

  val MNK_CMD     = 20.U(7.W)
  val COMPUTE_CMD = 21.U(7.W)

  //==========================================================================
  // dataflow configuration
  //==========================================================================
  val GARBAGE_ADDR      = "hffffffff".U(32.W)
  val OUTPUT_STATIONARY = 0.U(1.W)
  val WEIGHT_STATIONARY = 1.U(1.W)

  //==========================================================================
  // addressing mode for A,B,C,D operands
  //==========================================================================
  val ADDR_MODE_NORMAL = 0.U(1.W)
  val ADDR_MODE_IM2COL = 1.U(1.W)

  val MATRIX_ID_A = 0.U(2.W)
  val MATRIX_ID_B = 1.U(2.W)
  val MATRIX_ID_C = 2.U(2.W)
  val MATRIX_ID_D = 3.U(2.W)

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

object MatrixID extends Enumeration {
  val MATRIX_A, MATRIX_B, MATRIX_C, MATRIX_D = Value
}


