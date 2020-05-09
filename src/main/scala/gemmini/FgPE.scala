package gemmini

import chisel3._
import chisel3.util._

class FgPEControl extends Bundle {
  val prop = Bool()
}

class FgPE[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit ev: Arithmetic[T]) extends Module {
  import config._
  import ev._
  //=========================================================================
  // module interface
  //=========================================================================
  val io = IO(new Bundle {
    // non-broadcast shiftreg paths
    val in_a      = Input(inputType)
    val out_a     = Output(inputType)
    val in_d      = Input(outputType)
    val out_c     = Output(outputType)
    val in_b_idx  = Input(UInt(2.W))
    val out_b_idx = Output(UInt(2.W))
    val in_valid  = Input(Bool())
    val out_valid = Output(Bool())

    val in_b_fast        = Input(outputType)
    val out_b_fast       = Output(outputType)
    val in_b_idx_fast    = Input(UInt(2.W))
    val out_b_idx_fast   = Output(UInt(2.W))
    val in_b_valid_fast  = Input(Bool())
    val out_b_valid_fast = Output(Bool())
  })

  val bs = Reg(Vec(3, inputType))

  //------------------
  // slow path
  //------------------
  io.out_valid := io.in_valid
  io.out_b_idx := io.in_b_idx
  io.out_a     := io.in_a
  io.out_c     := io.in_d.mac(io.in_a, bs(io.in_b_idx))

  //------------------
  // fast path
  //------------------
  io.out_b_fast       := bs(io.in_b_idx_fast)
  io.out_b_idx_fast   := io.in_b_idx_fast
  io.out_b_valid_fast := io.in_b_valid_fast
  when (io.in_b_valid_fast) {
    bs(io.in_b_idx_fast) := io.in_b_fast
  }
}
