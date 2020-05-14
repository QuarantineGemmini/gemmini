
package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._

class MeshInner[T <: Data: Arithmetic](config: GemminiArrayConfig[T])
  (implicit val p: Parameters) extends Module {
  import config._
  //=========================================================================
  // module interface
  //=========================================================================
  val io = IO(new Bundle {
    // slow path
    val in_valid  = Input(Vec(DIM, Bool()))
    val in_a      = Input(Vec(DIM, inputType))
    val in_b_idx  = Input(Vec(DIM, UInt(2.W)))
    val out_valid = Output(Vec(DIM, Bool()))
    val out_c     = Output(Vec(DIM, accType))
    // fast path (no pipeline registers)
    val in_b_fast     = Input(Vec(DIM, inputType))
    val in_b_idx_fast = Input(Vec(DIM, UInt(2.W)))
  })

  //=========================================================================
  // body
  //=========================================================================
  // mesh(r)(c) => Tile at row r, column c
  val mesh = Seq.fill(DIM, DIM)(Module(new PE(config)))
  val meshT = mesh.transpose

  //------------------
  // slow path
  //------------------
  for (r <- 0 until DIM) {
    mesh(r).foldLeft(io.in_a(r)) {
      case (in_a, pe) =>
        pe.io.in_a := RegNext(in_a)
        pe.io.out_a
    }
  }
  for (c <- 0 until DIM) {
    meshT(c).foldLeft((0.U.asTypeOf(outputType),
                       io.in_b_idx(c),
                       io.in_valid(c))) {
      case ((in_d, in_b_idx, in_valid), pe) =>
        pe.io.in_d     := RegNext(in_d)
        pe.io.in_b_idx := RegNext(in_b_idx)
        pe.io.in_valid := RegNext(in_valid)
        (pe.io.out_c, pe.io.out_b_idx, pe.io.out_valid) 
    }
  }
  val last = mesh(DIM-1)
  io.out_valid := VecInit(last.map {pe => RegNext(pe.io.out_valid)})
  io.out_c := VecInit(last.map {pe => RegNext(pe.io.out_c.asTypeOf(accType))})

  //------------------
  // fast path
  //------------------
  for (c <- 0 until DIM) {
    meshT(c).foldLeft((RegNext(io.in_b_fast(c)), 
                       RegNext(io.in_b_idx_fast(c)),
                       RegNext(io.in_valid(c)))) {
      case ((in_b_fast, in_b_idx_fast, in_b_valid_fast), pe) =>
        pe.io.in_b_fast       := in_b_fast
        pe.io.in_b_idx_fast   := in_b_idx_fast
        pe.io.in_b_valid_fast := in_b_valid_fast
        (pe.io.out_b_fast, pe.io.out_b_idx_fast, pe.io.out_b_valid_fast)
    }
  }
}
