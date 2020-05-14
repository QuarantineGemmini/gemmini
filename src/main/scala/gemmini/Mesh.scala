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

class MeshQueueTag[T <: Data](val config: GemminiArrayConfig[T])
  (implicit val p: Parameters) extends Bundle with TagQueueTag {
  import config._
  val do_writeback = Bool()
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
  val wb_lrange = new LocalRange(config)

  override def make_this_garbage(dummy: Int = 0): Unit = {
    do_writeback := false.B
    wb_lrange.garbage := true.B
  }
}

class Mesh[T <: Data : Arithmetic](val config: GemminiArrayConfig[T])
  (implicit val p: Parameters)
  extends Module with HasCoreParameters {
    import config._

  //========================================================================
  // I/O interface
  //========================================================================
  val io = IO(new Bundle {
    val in_valid     = Input(Bool())
    val a            = Input(Vec(DIM, inputType))
    val b            = Input(Vec(DIM, inputType))
    val flipped      = Input(Bool())
    val tag_in       = Flipped(Decoupled(new MeshQueueTag(config)))
    val out_valid    = Output(Bool())
    val out          = Output(Vec(DIM, accType))
    val tag_out      = Output(new Bundle {
      val bits   = new MeshQueueTag(config)
      val commit = Bool()
    })
    val busy = Output(Bool())
  })

  //==========================================================================
  // flipping double-buffer logic
  //==========================================================================
  val b_idx   = RegInit(0.U(2.W))
  val b_idx_n = Mux(io.in_valid && io.flipped, 
                  Mux(b_idx === 2.U, 0.U, b_idx + 1.U), b_idx)
  b_idx := b_idx_n

  val b_idx_fast_n = Mux(b_idx_n === 2.U, 0.U, b_idx_n + 1.U)

  //=========================================================================
  // Create inner Mesh
  //=========================================================================
  val mesh = Module(new MeshInner(config))

  for(i <- 0 until DIM) {
    // slow path
    mesh.io.in_valid(i)      := ShiftRegister(io.in_valid, i)
    mesh.io.in_a(i)          := ShiftRegister(io.a(DIM-1-i), i)
    mesh.io.in_b_idx(i)      := ShiftRegister(b_idx_n, i)
    // fast path
    mesh.io.in_b_fast(i)     := ShiftRegister(io.b(i), i)
    mesh.io.in_b_idx_fast(i) := ShiftRegister(b_idx_fast_n, i)
  }

  //=========================================================================
  // mesh->ex-ctrller output
  //=========================================================================
  io.out_valid := ShiftRegister(mesh.io.out_valid(0), DIM-1)
  for(i <- 0 until DIM) {
    io.out(i) := ShiftRegister(mesh.io.out_c(i), DIM-1-i)
  }

  //=========================================================================
  // Tags
  //=========================================================================
  // tag is written to on the very last cycle of input-data
  val tag_queue = Queue(io.tag_in, 5)
  tag_queue.ready := false.B

  // this current tag allows delaying the written tag_in by 1 matmul, since
  // we load the tag for the preload, and have to write the tag out after
  // the FOLLOWING compute
  val garbage_tag = Wire(UDValid(new MeshQueueTag(config)))
  garbage_tag.pop()
  garbage_tag.bits := DontCare
  garbage_tag.bits.make_this_garbage()
  val current_tag = RegInit(garbage_tag)

  // we are busy if we still have unfinished, valid tags
  io.busy := tag_queue.valid || 
             (current_tag.valid && current_tag.bits.do_writeback)

  val output_counter = RegInit(0.U(DIM_IDX.W))
  val is_last_row_output = (output_counter === (DIM-1).U)
  output_counter := wrappingAdd(output_counter, io.out_valid, DIM)

  io.tag_out.commit := false.B
  io.tag_out.bits := current_tag.bits
  io.tag_out.bits.wb_lrange.row_start := 
                      current_tag.bits.wb_lrange.row_start + output_counter

  when (is_last_row_output && io.out_valid) {
    io.tag_out.commit := current_tag.valid
    tag_queue.ready := true.B
    current_tag.push(tag_queue.bits)
    assert(tag_queue.fire(), "mesh missing next tag!")
  }
}

