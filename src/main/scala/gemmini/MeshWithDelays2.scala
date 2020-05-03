//===========================================================================
// - this is the mesh with input/output shifters AND tag-queue management
// - the tag_in must be written on the last row of the matrix
// - each matrix MUST be DIM rows long right now (TODO: fix this)
// - the output is a Valid interface (it is not stallable!)
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import gemmini.Util._

class MeshQueueTag2[T <: Data: Arithmetic](config: GemminiArrayConfig[T])
  (implicit val p: Parameters) extends Bundle with TagQueueTag {
  import config._

  val rob_id = UDValid(UInt(log2Up(rob_entries).W))
  val rows = UInt(log2Up(FG_DIM + 1).W)
  val cols = UInt(log2Up(FG_DIM + 1).W)
  val fg_col_start = UInt()
  val bank_start = UInt()
  val banks = UInt()
  val garbage = Bool()
  val accum = Bool()

  override def make_this_garbage(dummy: Int = 0): Unit = {
    rob_id.valid := false.B
    addr.make_this_garbage()
  }
  override def cloneType: MeshQueueTag2.this.type
    = new MeshQueueTag2(config).asInstanceOf[this.type]
}


class MeshWithDelays2[T <: Data: Arithmetic](config: GemminiArrayConfig[T])
  (implicit val p: Parameters)
  extends Module with HasCoreParameters {
  import config._
  //=========================================================================
  // module interface
  //=========================================================================
  val DIM = FG_DIM*tileRows

  val A_TYPE = Vec(FG_DIM, Vec(tileRows, inputType))
  val D_TYPE = Vec(FG_DIM, Vec(tileCols, inputType))
  val C_TYPE = Vec(FG_DIM, Vec(tileCols, outputType))
  val B_TYPE = Vec(FG_DIM, Vec(tileCols, inputType))

  val io = IO(new Bundle {
    val a       = Flipped(Decoupled(A_TYPE))
    val b       = Flipped(Decoupled(B_TYPE))
    val tag_in  = Flipped(Decoupled(new MeshQueueTag(config)))
    val pe_ctrl = Input(new PEControl(accType))
    val out     = Valid(C_TYPE)
    val tag_out = Output(new MeshQueueTag(config))
    val busy    = Output(Bool())
    val prof    = Input(new Profiling)
  })

  //=========================================================================
  // create triangle-shaped shifter pipeline for the signals
  //=========================================================================
  def shifted[T <: Data](
    x: Vec[Vec[T]], banks: Int, reverse: Boolean = false) = {
    assert(x.size % banks == 0, "cannot bank without clean divisors")

    val banked_len = x.size / banks
    val banked_x   = x.grouped(banked_len).toSeq

    val indexes = if (reverse) banked_x.indices.reverse
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
  // buffer inputs A, D when they arrive out of order
  //==========================================================================
  val a_buf = RegEnable(io.a.bits, io.a.fire())
  val d_buf = RegEnable(io.d.bits, io.d.fire())

  val a_written = RegInit(false.B)
  val d_written = RegInit(false.B)

  io.a.ready := !a_written
  io.d.ready := !d_written

  val a_written_n = a_written || io.a.fire()
  val d_written_n = d_written || io.d.fire()
  val pause = !(a_written_n && d_written_n)

  a_written := Mux(pause, a_written_n, false.B)
  d_written := Mux(pause, d_written_n, false.B)

  // TODO: enable row-count that is not DIM
  val fire_row_index = RegInit(0.U(log2Up(DIM+1).W))
  fire_row_index := wrappingAdd(fire_row_index, !pause, DIM)

  //==========================================================================
  // flipping double-buffer logic
  //==========================================================================
  val propagate_index   = Reg(UInt(1.W))
  val propagate_index_n = WireInit(propagate_index)

  when(!pause && (fire_row_index === 0.U)) {
    propagate_index_n := io.pe_ctrl.propagate ^ propagate_index
    propagate_index := propagate_index_n
  }

  //=========================================================================
  // Create inner Mesh
  //=========================================================================
  val mesh = Module(new Mesh2(config))

  val not_paused_vec = VecInit(Seq.fill(FG_DIM)(
                         VecInit(Seq.fill(tileCols)(!pause))))
  val a_shifter_in   = Mux(io.a.fire(), io.a.bits, a_buf)
  val d_shifter_in   = Mux(io.d.fire(), io.d.bits, d_buf)

  mesh.io.in_valid := shifted(not_paused_vec, shifter_banks)
  mesh.io.in_a     := shifted(a_shifter_in, shifter_banks)
  mesh.io.in_d     := shifted(d_shifter_in, shifter_banks)

  // just fill b-cols with nothing, since in WS mode, we always accumulate
  // by writing the D-matrix to the accumulator
  mesh.io.in_b := 0.U.asTypeOf(B_TYPE)

  mesh.io.in_ctrl.zipWithIndex.foreach { case (ss, i) =>
    ss.foreach(_.propagate := ShiftRegister(propagate_index_n, i))
  }

  //=========================================================================
  // mesh->ex-ctrller output
  //=========================================================================
  val shifter_out = mesh.io.out_b
  io.out.valid := shifted(mesh.io.out_valid, shifter_banks, true)(0)(0)
  io.out.bits  := shifted(shifter_out, shifter_banks, true)

  //=========================================================================
  // Tags
  //=========================================================================
  // tag is written to on the very last cycle of input-data
  val tag_queue = Queue(io.tag_in, 5)
  tag_queue.ready := false.B

  // this current tag allows delaying the written tag_in by 1 matmul, since
  // we load the tag for the preload, and have to write the tag out after
  // the FOLLOWING compute
  val garbage_tag = WireInit(0.U.asTypeOf(new MeshQueueTag(config)))
  garbage_tag.make_this_garbage()
  val current_tag = RegInit(garbage_tag)
  io.tag_out := current_tag

  // we are busy if we still have unfinished, valid tags
  io.busy := tag_queue.valid || current_tag.rob_id.valid

  // TODO: this is hardcoded to output DIM rows
  val output_counter = RegInit(0.U(log2Up(DIM + 1).W))
  val is_last_row_output = (output_counter === (DIM-1).U)
  output_counter := wrappingAdd(output_counter, io.out.valid, DIM)

  when (is_last_row_output && io.out.valid) {
    tag_queue.ready := true.B
    current_tag := Mux(tag_queue.fire(),tag_queue.bits, garbage_tag)
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
