//===========================================================================
// - this module is the level of hierarchy between the fine-grained execute
// controller and the fine-grained meshes themselves
//===========================================================================
package gemmini

import scala.math.{pow}
import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import gemmini.Util._
import GemminiISA._

class FgMeshQueueTag[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit val p: Parameters) extends Bundle with TagQueueTag {
  import config._
  val valid = Bool() // false if just a dummy when COMPUTE only (not PRELOAD)
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
    val a_mux_sel  = Input(UInt(FG_NUM_CTR_CTR.W))
    val b_mux_sel  = Input(UInt(FG_NUM_CTR_CTR.W))
    val flipped    = Input(Bool())
    val tag_in     = Flipped(Decoupled(new FgMeshQueueTag(config)))
    val out_valid  = Output(Bool())
    val out        = Output(Vec(FG_NUM, Vec(FG_DIM, accType)))
    val tag_out    = Valid(new FgMeshQueueTag(config))
    val busy       = Output(Bool())
    val prof       = Input(new Profiling)
  })

  val fg_mesh = Seq.fill(FG_NUM)(Module(new FgMeshWithDelays(config)))

  // Each sub-mesh selects from a set of values that are indexed into various 
  // portiions of the a and b inputs based upon the fine-grainedness of the 
  // array, the size of the array, and the sub-meshes position in the full mesh
  val a_mesh_muxes = Wire(Vec(FG_NUM, Vec(FG_NUM_CTR, Vec(FG_DIM,inputType))))
  val b_mesh_muxes = Wire(Vec(FG_NUM, Vec(FG_NUM_CTR, Vec(FG_DIM,inputType))))
  val fg_pow2s = (0 to log2Up(FG_NUM)).map { e=>pow(2,e).toInt }

  // Routing the possible inputs to each sub-array's mux
  for (i <- 0 until SQRT_FG_NUM) {
    for (j <- 0 until SQRT_FG_NUM) {
      //this must be floordiv
      val idx_divs = fg_pow2s.map{e => (i*SQRT_FG_NUM+j)/e}.reverse 
      for (k <- 0 until FG_NUM_CTR) {
        a_mesh_muxes(i*SQRT_FG_NUM + j)(k) := io.a(idx_divs(k))
        b_mesh_muxes(j*SQRT_FG_NUM + i)(k) := io.b(idx_divs(k))
      }
    }
  }

  for (i <- 0 until FG_NUM) {
    fg_mesh(i).io.in_valid  := io.in_valid
    fg_mesh(i).io.a         := a_mesh_muxes(i)(io.a_mux_sel)
    fg_mesh(i).io.b         := b_mesh_muxes(i)(io.b_mux_sel)
    fg_mesh(i).io.flipped   := io.flipped
    io.out(i)               := fg_mesh(i).io.out
  }
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
  garbage_tag.bits := DontCare
  garbage_tag.pop()
  garbage_tag.bits.make_this_garbage()
  val current_tag = RegInit(garbage_tag)
  io.tag_out.valid := false.B
  io.tag_out.bits := current_tag.bits

  // we are busy if we still have unfinished, valid tags
  io.busy := tag_queue.valid || current_tag.valid

  val output_counter = RegInit(0.U(FG_DIM_IDX.W))
  val is_last_row_output = (output_counter === (FG_DIM-1).U)
  output_counter := wrappingAdd(output_counter, io.out_valid, FG_DIM)

  when (is_last_row_output && io.out_valid) {
    io.tag_out.valid := current_tag.valid
    current_tag.pop()
    tag_queue.ready := true.B
    when (tag_queue.fire() && tag_queue.bits.valid) {
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

