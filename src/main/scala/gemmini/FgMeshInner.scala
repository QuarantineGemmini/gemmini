
package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._

class FgMeshInner[T <: Data: Arithmetic](config: FgGemminiArrayConfig[T])
  (implicit val p: Parameters) extends Module {
  import config._
  //=========================================================================
  // module interface
  //=========================================================================
  val io = IO(new Bundle {
    val in_valid  = Input(Vec(FG_DIM, Bool()))
    val in_a      = Input(Vec(FG_DIM, inputType))
    val in_b      = Input(Vec(FG_DIM, inputType))
    val in_ctrl   = Input(Vec(FG_DIM, new FgPEControl))
    val out_valid = Output(Vec(FG_DIM, Bool()))
    val out_c     = Output(Vec(FG_DIM, accType))
  })

  //=========================================================================
  // body
  //=========================================================================
  // mesh(r)(c) => Tile at row r, column c
  val mesh = Seq.fill(FG_DIM, FG_DIM)(Module(new FgPE(config)))
  val meshT = mesh.transpose

  for (r <- 0 until FG_DIM) {
    mesh(r).foldLeft(io.in_a(r)) {
      case (in_a, pe) =>
        pe.io.in_a := RegNext(in_a)
        pe.io.out_a
    }
  }
  for (c <- 0 until FG_DIM) {
    meshT(c).foldLeft((io.in_b(c), 
                       0.U.asTypeOf(outputType),
                       io.in_ctrl(c),
                       io.in_valid(c))) {
      case ((in_b, out_c, in_ctrl, valid), pe) =>
        pe.io.in_b      := RegEnable(in_b, valid)
        pe.io.in_d      := RegEnable(out_c, valid)
        pe.in_ctrl.prop := RegEnable(ctrl.prop, valid)
        (pe.io.out_b, pe.io.out_c, pe.io.out_ctrl, pe.io.out_valid)
    }
  }

  io.out_valid := VecInit(meshT.map {pe => RegInit(pe.out_valid)})
  io.out_c := VecInit(meshT.map {pe => RegInit(pe.out_c.asTypeOf(accType))})
}
