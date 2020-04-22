//===========================================================================
// BeatMerger
// - shifts/aligns and merges multiple beats from multiple transactions 
// into a single buffer. used during DMA reads into scratchpad
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import Util._

class FgBeatMerger[T <: Data](config: GemminiArrayConfig[T], max_bytes: Int)
  (implicit p: Parameters) extends Module {
  import config._

  // I/O interface
  val io = IO(new Bundle {
    val peek  = new FgXactTrackerPeekIO(config)
    val beat  = Flipped(Decoupled(FgStreamBeatData(config)))
    val resp  = Decoupled(new FgStreamReadResponse(max_bytes))
  })

  // internal logic/state
  val buffer = RegInit(0.U((max_bytes*8).W))
  val lrange = RegInit(0.U.asTypeOf[FgLocalRange])
  val rob_id = RegInit(UInt(LOG2_ROB_ENTRIES.W))

  io.peek.pop    := false.B
  io.peek.xactid := io.beat.bits.xactid

  // TODO: possibly pipeline this massive barrel shifter if its too slow
  val lrange_next  = io.peek.entry.lrange
  val rob_id_next  = io.peek.entry.rob_id
  val rshift_bytes = io.peek.entry.start_rshift
  val lshift_bytes = io.beat.bits.beat_lshift + io.beat.bits.txn_lshift
  val lshift_bits  = Cat(lshift_bytes - rshift_bytes, 0.U(3.W))
  val shifted_data = io.beat.bits.data << lshfit_bits)
  val buffer_next  = buffer | shifted_data

  // ready to merge in a new beat
  io.beat.ready := false.B

  // output merged buffer
  io.resp.valid  := false.B
  io.resp.data   := buffer
  io.resp.lrange := lrange
  io.resp.rob_id := rob_id

  // FSM
  val (s_MERGE_BEAT :: s_OUTPUT_DATA :: Nil) = Enum(2)
  val state = RegInit(s_MERGE_BEAT)

  switch (state) {
    is (s_MERGE_BEAT) {
      io.beat.ready := true.B
      when (io.beat.fire()) {
        assert(lshift_bytes >= rshift_bytes, 
               "cannot rshift the beat data more than lshift")
        buffer := buffer_next
        lrange := lrange_next
        rob_id := rob_id_next
        io.peek.pop := io.beat.bits.is_last_beat
        when (io.beat.bits.is_last_beat && io.beat.bits.is_last_txn) {
          state := s_OUTPUT_DATA
        }
      }
    }
    is (s_OUTPUT_DATA) {
      io.resp.valid := true.B
      when (io.resp.fire()) {
        buffer := 0.U
        state := s_MERGE_BEAT
      }
    }
  }
}
