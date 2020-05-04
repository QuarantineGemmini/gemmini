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
  val prop_index   = RegInit(false.B)
  val prop_index_n = Mux(io.in_valid && io.flipped, ~prop_index, prop_index)
  prop_index := prop_index_n 

  //=========================================================================
  // Create inner Mesh
  //=========================================================================
  val mesh = Module(new FgMeshInner(config))

  for(i <- 0 until FG_DIM) {
    mesh.io.in_valid(i)     := ShiftRegister(io.in_valid, i)
    mesh.io.in_a(i)         := ShiftRegister(io.a(i), i)
    mesh.io.in_b(i)         := ShiftRegister(io.b(i), i)
    mesh.io.in_ctrl(i).prop := ShiftRegister(prop_index_n, i)
  }

  //=========================================================================
  // mesh->ex-ctrller output
  //=========================================================================
  io.out_valid := ShiftRegister(mesh.io.out_valid(0), FG_DIM-1)
  for(i <- 0 until FG_DIM) {
    io.out(i) := ShiftRegister(mesh.io.out_c(i), FG_DIM-1-i)
  }
}
