//===========================================================================
// - this module is the level of hierarchy between the fine-grained execute
// controller and the fine-grained meshes themselves
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import gemmini.Util._
import GemminiISA._

class FgMeshQueueTag[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit val p: Parameters) extends Bundle with TagQueueTag {
  import config._
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
  val wb_lrange = new FgLocalRange(config)

  override def make_this_garbage(dummy: Int = 0): Unit = {
    wb_lrange.garbage := true.B
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
    val a_mux_ctrl = Input(Vec(mesh_partition_list.length, Bool())) //TODO change to a global param?
    val b_mux_ctrl = Input(Vec(mesh_partition_list.length, Bool()))
    val flipped    = Input(Bool())
    val tag_in     = Flipped(Decoupled(new FgMeshQueueTag(config)))
    val out_valid  = Output(Bool())
    val out        = Output(Vec(FG_NUM, Vec(FG_DIM, accType)))
    val tag_out    = Output(new FgMeshQueueTag(config))
    val busy       = Output(Bool())
    val prof       = Input(new Profiling)
  })

  val fg_mesh = Seq.fill(FG_NUM)(Module(new FgMeshWithDelays(config)))

  //TODO: check this is legal/functionally correct
  //Convert thermometer to binary (00000 -> 000; 10000 -> 001; 11100 -> 011)
  val a_mux_sel = io.a_mux_ctrl.map{ b => b.asUInt }.reduce(_ + _)
  val b_mux_sel = io.b_mux_ctrl.map{ b => b.asUInt }.reduce(_ + _)

  val mesh_partition_list = (0 to log2Up(FG_NUM)).map { e=>pow(2,e).toInt }

  // Each sub-mesh selects from a set of values that are indexed into various portiions of
  // the a and b inputs based upon the fine-grainedness of the array, the size of the array,
  // and the sub-meshes position in the full mesh
  val a_mesh_muxes = Wire(Vec(FG_NUM, Vec(mesh_partition_list.length, Vec(FG_DIM, inputType))))
  val b_mesh_muxes = Wire(Vec(FG_NUM, Vec(mesh_partition_list.length, Vec(FG_DIM, inputType))))

  var idx_divs = 0 //TODO better way to do this (gets reassigned in the for loop)
  // Routing the possible inputs to each sub-array
  for (i <- 0 until SQRT_FG_NUM) {
    for (j <- 0 until SQRT_FG_NUM) {
      idx_divs = mesh_partition_list.map{ e => (i*SQRT_FG_NUM+j) / e }.reverse //this must be floordiv
      for (k <- 0 until mesh_partition_list.length) {
        a_mesh_muxes(i*SQRT_FG_NUM + j)(k) := io.a(idx_divs(k))
        b_mesh_muxes(j*SQRT_FG_NUM + i)(k) := io.b(idx_divs(k))
      }
    }
  }


  for (i <- 0 until FG_NUM) {
    fg_mesh(i).io.in_valid  := io.in_valid
    fg_mesh(i).io.a         := a_mesh_muxes(i)(a_mux_sel)
    fg_mesh(i).io.b         := b_mesh_muxes(i)(b_mux_sel)
    fg_mesh(i).io.flipped   := io.flipped

    io.out(i)               := fg_mesh(i).io.out
  }

  //TODO is this alright?
  io.out_valid := fg_mesh(0).io.out_valid


  //=========================================================================
  // Tags
  //=========================================================================
  // tag is written to on the very last cycle of input-data
  val tag_queue = Queue(io.tag_in, 5)
  tag_queue.ready := false.B

  // this current tag allows delaying the written tag_in by 1 matmul, since
  // we load the tag for the preload, and have to write the tag out after
  // the FOLLOWING compute
  val garbage_tag = Wire(UDValid(new FgMeshQueueTag(config)))
  garbage_tag := DontCare
  garbage_tag.pop()
  garbage_tag.bits.make_this_garbage()
  val current_tag = RegInit(garbage_tag)
  io.tag_out := current_tag.bits

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

