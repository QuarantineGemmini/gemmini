package gemmini

import chisel3._

object GemminiISA {
  // funct values
  val CONFIG_CMD = 0.U
  val LOAD_CMD = 2.U
  val STORE_CMD = 3.U
  val COMPUTE_AND_FLIP_CMD = 4.U
  val COMPUTE_AND_STAY_CMD = 5.U
  val PRELOAD_CMD = 6.U
  val FLUSH_CMD = 7.U
  val LOOP_WS = 8.U

  // Gemmini2 Opcodes
  val ADDR_AB  = 10.U
  val ADDR_CD  = 11.U
  val SIZE0    = 12.U
  val SIZE1    = 13.U
  val RPT_BIAS = 14.U
  val RESET    = 15.U

  // rs1[1:0] values
  val CONFIG_LOAD = 1.U
  val CONFIG_STORE = 2.U
  val CONFIG_EX = 0.U
}
