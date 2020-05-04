//===========================================================================
// - this module is the level of hierarchy between the fine-grained execute
// controller and the fine-grained meshes themselves
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip,tile._
import gemmini.Util._
import GemminiISA._


class FgMesh[T <: Data : Arithmetic](val config: FgGemminiArrayConfig[T])
  (implicit val p: Parameters)
  extends Module with HasCoreParameters {
    import config._

  val ROW_TYPE = Vec(FG_NUM, Vec(FG_DIM, inputType))
  val COL_TYPE = Vec(FG_NUM, Vec(FG_DIM, inputType))

  //========================================================================
  // I/O interface
  //========================================================================
  val io = IO(new Bundle {
    val a          = Flipped(Valid(ROW_TYPE))
    val b          = Flipped(Valid(COL_TYPE))
    val a_mux_ctrl = UInt(FG_NUM.W)
    val b_mux_ctrl = UInt(FG_NUM.W)
    val tag_in     = Flipped(Decoupled(new FgMeshQueueTag(config)))
    val pe_ctrl    = Input(new PEControl(accType))
    val out        = Valid(COL_TYPE)
    val tag_out    = Output(new FgMeshQueueTag(config))
    val busy       = Output(Bool())
    val prof       = Input(new Profiling)
  }

  val fg_mesh = Seq.fill(FG_NUM)(Module(new FgMeshWithDelays(config)))

  val a_mux_seq = Seq.fill(FG_NUM)(Vec(FG_DIM, inputType)))
  val b_mux_seq = Seq.fill(FG_NUM)(Vec(FG_DIM, inputType)))

  // Connect up all of the muxing of inputs
  for (i <- 0 until SQRT_FG_NUM) {
    for (j <- 0 until SQRT_FG_NUM) {
      if (i == 0 && j == 0) {
        a_mux_seq(0) := io.a.bits(0)
        b_mux_seq(0) := io.b.bits(0)
      } else if ((j*SQRT_FG_NUM + i) % FG_NUM == 0) {
        a_mux_seq(j*SQRT_FG_NUM + i) := Mux(a_mux_ctrl(j*SQRT_FG_NUM + i),
                                            io.a.bits(j*SQRT_FG_NUM + i),
                                            a_mux_seq((j-1)*SQRT_FG_NUM + i))
        b_mux_seq(j*SQRT_FG_NUM + i) := Mux(b_mux_ctrl(j*SQRT_FG_NUM + i),
                                            io.b.bits(j*SQRT_FG_NUM + i),
                                            b_mux_seq((j-1)*SQRT_FG_NUM + i))
      } else {
        a_mux_seq(j*SQRT_FG_NUM + i) := Mux(a_mux_ctrl(j*SQRT_FG_NUM + i),
                                            io.a.bits(j*SQRT_FG_NUM + i),
                                            a_mux_seq(j*SQRT_FG_NUM + i - 1))
        b_mux_seq(j*SQRT_FG_NUM + i) := Mux(b_mux_ctrl(j*SQRT_FG_NUM + i),
                                            io.b.bits(j*SQRT_FG_NUM + i),
                                            b_mux_seq(j*SQRT_FG_NUM + i - 1))
      }
    }
  }

  for (i <- 0 until SQRT_FG_NUM) {
    for (j <- 0 until SQRT_FG_NUM) {
      fg_mesh(j*SQRT_FG_NUM + i).io.a.bits  := a_mux_seq(j*SQRT_FG_NUM + i)
      fg_mesh(j*SQRT_FG_NUM + i).io.a.valid := io.a.valid
      fg_mesh(j*SQRT_FG_NUM + i).io.b.bits := b_mux_seq(j*SQRT_FG_NUM + i)
      fg_mesh(j*SQRT_FG_NUM + i).io.b.valid := io.b.valid
      fg_mesh(j*SQRT_FG_NUM + i).io.pe_ctrl := io.pe_ctrl
      fg_mesh(j*SQRT_FG_NUM + i).io.prof    := io.prof

      io.out.bits(j*SQRT_FG_NUM + i) := fg_mesh(j*SQRT_FG_NUM + i).io.out.bits
    }
  }


  //TODO move firing logic out of sub-arrays into this hierarchy level
  io.a.ready := fg_mesh(0).io.a.ready
  io.b.ready := fg_mesh(0).io.b.ready

  io.out.valid := fg_mesh(0).io.out.valid
  // Add muxing of inputs
//  fg_mesh(0).io.a <> io.a(0)
//  fg_mesh(0).io.b <> io.b(0)
//  for (i <- 1 until FG_NUM) {
//    for (j <- 1 until FG_NUM) {
//      fg_mesh(j*FG_NUM + i).io.a <> io.a(j*FG_NUM + i)
//      fg_mesh(j*FG_NUM + i).io.b <> io.b(j*FG_NUM + i)
//    }
//  }
//
  //=========================================================================
  // Tags
  //=========================================================================
  // tag is written to on the very last cycle of input-data
  val tag_queue = Queue(io.tag_in, 5)
  tag_queue.ready := false.B

  // this current tag allows delaying the written tag_in by 1 matmul, since
  // we load the tag for the preload, and have to write the tag out after
  // the FOLLOWING compute
  val garbage_tag = WireInit(UDValid(new FgMeshQueueTag(config)))
  garbage_tag.pop()
  garbage_tag.bits.make_this_garbage()
  val current_tag = RegInit(garbage_tag)
  io.tag_out := garbage_tag

  // we are busy if we still have unfinished, valid tags
  io.busy := tag_queue.valid || current_tag.valid

  // TODO: this is hardcoded to output DIM rows
  val output_counter = RegInit(0.U(FG_DIM_IDX.W))
  val is_last_row_output = (output_counter === (FG_DIM-1).U)
  output_counter := wrappingAdd(output_counter, io.out.valid, FG_DIM)

  when (is_last_row_output && io.out.valid) {
    tag_queue.ready := true.B
    current_tag := Mux(tag_queue.fire(), tag_queue.bits, garbage_tag)
  }
}

