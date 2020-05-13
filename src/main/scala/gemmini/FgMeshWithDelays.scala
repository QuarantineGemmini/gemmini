package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import gemmini.Util._

class FgMeshWithDelays[T <: Data:Arithmetic](config: FgGemminiArrayConfig[T])
  (implicit val p: Parameters) extends Module with HasCoreParameters {
  import config._
  //=========================================================================
  // module interface
  //=========================================================================
  val io = IO(new Bundle {
    val in_valid  = Input(Bool())
    val a         = Input(Vec(FG_DIM, inputType))
    val b         = Input(Vec(FG_DIM, inputType))
    val flipped   = Input(Bool())
    val out_valid = Output(Bool())
    val out       = Output(Vec(FG_DIM, accType))
  })

  //==========================================================================
  // flipping double-buffer logic
  //==========================================================================
  val b_idx = RegInit(0.U(2.W))
  val b_idx_n = Mux(io.in_valid && io.flipped, 
                  Mux(b_idx === 2.U, 0.U, b_idx + 1.U), b_idx)
  b_idx := b_idx_n

  val b_idx_fast_n = Mux(b_idx_n === 2.U, 0.U, b_idx_n + 1.U)

  //=========================================================================
  // Create inner Mesh
  //=========================================================================
  val mesh = Module(new FgMeshInner(config))

  for(i <- 0 until FG_DIM) {
    // slow path
    mesh.io.in_valid(i)      := ShiftRegister(io.in_valid, i)
    mesh.io.in_a(i)          := ShiftRegister(io.a(FG_DIM-1-i), i)
    mesh.io.in_b_idx(i)      := ShiftRegister(b_idx_n, i)
    // fast path
    mesh.io.in_b_fast(i)     := ShiftRegister(io.b(i), i)
    mesh.io.in_b_idx_fast(i) := ShiftRegister(b_idx_fast_n, i)
  }

  //=========================================================================
  // mesh->ex-ctrller output
  //=========================================================================
  io.out_valid := ShiftRegister(mesh.io.out_valid(0), FG_DIM-1)
  for(i <- 0 until FG_DIM) {
    io.out(i) := ShiftRegister(mesh.io.out_c(i), FG_DIM-1-i)
  }
}
