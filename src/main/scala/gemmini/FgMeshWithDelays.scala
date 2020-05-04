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
    val a         = Input(Vec(FG_NUM, Vec(FG_DIM, inputType)))
    val b         = Input(Vec(FG_NUM, Vec(FG_DIM, inputType)))
    val flip      = Input(Bool())
    val out_valid = Output(Bool())
    val out       = Output(Vec(FG_NUM, Vec(FG_DIM, accType)))
  })

  //=========================================================================
  // create triangle-shaped shifter pipeline for the signals
  //=========================================================================
  def shifted[T <: Data](x: Vec[Vec[T]], reverse: Boolean = false) = {
    val banked_len = x.size
    val banked_x   = x.grouped(banked_len).toSeq
    val indexes    = if (reverse) banked_x.indices.reverse
                     else         banked_x.indices

    (banked_x zip indexes).flatMap { case (bx, i) =>
      val bxVec = VecInit(bx)
      val sram_shift = i * banked_len
      val SRAMShifted = Shifter(bxVec, sram_shift, true.B, true)
      val indexes = if (reverse) SRAMShifted.indices.reverse
                    else         SRAMShifted.indices
      val RegShifted = (SRAMShifted zip indexes).map {
        case (srs, j) => ShiftRegister(srs, j)
      }
      RegShifted
    }
  }

  //==========================================================================
  // flipping double-buffer logic
  //==========================================================================
  val prop_index   = RegInit(false.B)
  val prop_index_n = Mux((io.in_valid && io.flip), ~prop_index, prop_index)
  prop_index := prop_index_n 

  //=========================================================================
  // Create inner Mesh
  //=========================================================================
  val mesh = Module(new FgMeshInner(config))

  mesh.io.in_valid := shifted(VecInit(Seq.fill(FG_DIM)(io.in_valid)))
  mesh.io.in_a     := shifted(io.a)
  mesh.io.in_b     := shifted(io.b)

  mesh.io.in_ctrl.zipWithIndex.foreach { case (ss, i) =>
    ss.foreach(_.prop := ShiftRegister(prop_index_n, i))
  }

  //=========================================================================
  // mesh->ex-ctrller output
  //=========================================================================
  io.out_valid := shifted(mesh.io.out_valid, true)(0)
  io.out       := shifted(mesh.io.out_c,     true)
}
