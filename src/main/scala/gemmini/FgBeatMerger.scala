//===========================================================================
// BeatMerger
// - shifts/aligns and merges multiple beats from multiple transactions 
// into a single buffer. used during DMA reads into scratchpad
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import Util._

class FgBeatMerger[T <: Data]
  (config: FgGemminiArrayConfig[T], max_xfer_bytes: Int)
  (implicit p: Parameters) extends Module {
  import config._
  //-----------------------------------------------
  // I/O interface
  //-----------------------------------------------
  val io = IO(new Bundle {
    val peek  = new FgDMATrackerPeekIO(config, max_xfer_bytes)
    val pop   = Output(Valid(UInt(DMA_REQS_IDX.W)))
    val beat  = Flipped(Decoupled(new Bundle {
      val xactid       = UInt(DMA_REQS_IDX.W)
      val data         = UInt(DMA_BUS_BITS.W)
      val beat_idx     = UInt(DMA_TXN_BEATS_IDX.W)
      val is_last_beat = Bool()
    }))
    val resp  = Decoupled(new FgDMALoadResponse(max_xfer_bytes))
  })

  //-----------------------------------------------
  // internal logic/state
  //-----------------------------------------------
  val buffer = RegInit(0.U((max_xfer_bytes*8).W))
  val lrange = RegInit(0.U.asTypeOf(new FgLocalRange(config)))
  val rob_id = RegInit(UInt(ROB_ENTRIES_IDX.W))

  io.pop.valid   := false.B
  io.pop.xactid  := io.beat.bits.xactid
  io.peek.xactid := io.beat.bits.xactid

  // TODO: possibly pipeline this massive barrel shifter if its too slow
  val lrange_next    = io.peek.entry.lrange
  val rob_id_next    = io.peek.entry.rob_id
  val is_last_txn    = io.peek.entry.is_last_txn
  val is_last_beat   = io.beat.bits.is_last_beat
  val data_start_idx = io.peek.entry.data_start_idx
  val beat_start_idx = io.beat.bits.beat_idx * DMA_BEAT_BYTES +
                       io.beat.bits.txn_start_idx
  val data           = io.beat.bits.data
  val data_shifted   = Mux(beat_start_idx > data_start_idx,
                           data << ((beat_start_idx - data_start_idx)*8.U),
                           data >> ((data_start_idx - beat_start_idx)*8.U))
  val buffer_next  = buffer | data_shifted

  // ready to merge in a new beat
  io.beat.ready := false.B

  // output merged buffer
  io.resp.valid  := false.B
  io.resp.data   := buffer
  io.resp.lrange := lrange
  io.resp.rob_id := rob_id

  //-----------------------------------------------
  // FSM
  //-----------------------------------------------
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
        io.peek.pop := is_last_beat        
        when (is_last_beat && is_last_txn) {
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

