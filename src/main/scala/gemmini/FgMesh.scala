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

class FgMeshQueueTag[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit val p: Parameters) extends Bundle with TagQueueTag {
  import config._
  val rob_id   = UInt(ROB_ENTRIES_IDX.W)
  val wb_lrange = new LocalRange(config)

  override def make_this_garbage(dummy: Int = 0): Unit = {
    c_lrange.garbage := true.B
  }
}

class FgMesh[T <: Data : Arithmetic](val config: FgGemminiArrayConfig[T])
  (implicit val p: Parameters)
  extends Module with HasCoreParameters {
    import config._

  //========================================================================
  // I/O interface
  //========================================================================
  val io = IO(new Bundle {
    val in_valid   = Input(Bool())
    val a          = Input(Vec(FG_NUM, Vec(FG_DIM, inputType)))
    val b          = Input(Vec(FG_NUM, Vec(FG_DIM, inputType)))
    val a_mux_ctrl = Input(UInt(FG_NUM.W))
    val b_mux_ctrl = Input(UInt(FG_NUM.W))
    val flip       = Input(bool)
    val tag_in     = Flipped(Decoupled(new FgMeshQueueTag(config)))
    val out_valid  = Output(Bool())
    val out        = Output(Vec(FG_NUM, Vec(FG_DIM, accType)))
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
                                            io.a(j*SQRT_FG_NUM + i),
                                            a_mux_seq((j-1)*SQRT_FG_NUM + i))
        b_mux_seq(j*SQRT_FG_NUM + i) := Mux(b_mux_ctrl(j*SQRT_FG_NUM + i),
                                            io.b(j*SQRT_FG_NUM + i),
                                            b_mux_seq((j-1)*SQRT_FG_NUM + i))
      } else {
        a_mux_seq(j*SQRT_FG_NUM + i) := Mux(a_mux_ctrl(j*SQRT_FG_NUM + i),
                                            io.a(j*SQRT_FG_NUM + i),
                                            a_mux_seq(j*SQRT_FG_NUM + i - 1))
        b_mux_seq(j*SQRT_FG_NUM + i) := Mux(b_mux_ctrl(j*SQRT_FG_NUM + i),
                                            io.b(j*SQRT_FG_NUM + i),
                                            b_mux_seq(j*SQRT_FG_NUM + i - 1))
      }
    }
  }

  for (i <- 0 until SQRT_FG_NUM) {
    for (j <- 0 until SQRT_FG_NUM) {
      fg_mesh(j*SQRT_FG_NUM + i).io.in_valid := io.in_valid
      fg_mesh(j*SQRT_FG_NUM + i).io.a    := a_mux_seq(j*SQRT_FG_NUM + i)
      fg_mesh(j*SQRT_FG_NUM + i).io.b    := b_mux_seq(j*SQRT_FG_NUM + i)
      fg_mesh(j*SQRT_FG_NUM + i).io.flip := io.flip
      io.out(j*SQRT_FG_NUM + i)          := fg_mesh(j*SQRT_FG_NUM + i).io.out
    }
  }
  io.out.valid := fg_mesh(0).io.out_valid

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
  val current_tag = Reg(garbage_tag)
  io.tag_out := current_tag

  // we are busy if we still have unfinished, valid tags
  io.busy := tag_queue.valid || current_tag.valid

  val output_counter = RegInit(0.U(FG_DIM_IDX.W))
  val is_last_row_output = (output_counter === (FG_DIM-1).U)
  output_counter := wrappingAdd(output_counter, io.out_valid, FG_DIM)

  when (is_last_row_output && io.out_valid) {
    current_tag.pop()
    tag_queue.ready := true.B
    when (tag_queue.fire()) {
      current_tag.push(tag_queue.bits)
    }
  }

  //=========================================================================
  // hardware profiling counters (non architecturally visible!)
  //=========================================================================
  //withReset(io.prof.start) {
  //  val flush_cycles        = RegInit(0.U(32.W))
  //  val wait_for_tag_cycles = RegInit(0.U(32.W))
  //  val col_valids          = RegInit(VecInit(Seq.fill(DIM)(0.U(32.W))))
  //
  //  flush_cycles := flush_cycles + flushing
  //  wait_for_tag_cycles := wait_for_tag_cycles + waiting_on_tag_in
  //  for(i <- 0 until FG_DIM; j <- 0 until tileCols) {
  //    col_valids(i*tileCols+j) := col_valids(i*tileCols+j) +
  //                                mesh.io.in_valid(i)(j)
  //  }

  //  when(io.prof.end) {
  //    printf(s"G2-PERF[%d]: mesh-flush-cycles: %d\n",
  //            io.prof.debug_cycle, flush_cycles)
  //    printf(s"G2-PERF[%d]: wait-for-tag-cycles: %d\n",
  //            io.prof.debug_cycle, wait_for_tag_cycles)
  //    for(i <- 0 until FG_DIM; j <- 0 until tileCols) {
  //      printf(s"G2-PERF[%d]: in-valid-cycles(%d)(%d): %d\n",
  //              io.prof.debug_cycle, i.U, j.U, col_valids(i*tileCols+j))
  //    }
  //  }
  //}

}

