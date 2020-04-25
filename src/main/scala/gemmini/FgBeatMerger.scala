//===========================================================================
// BeatMerger
// - shifts/aligns and merges multiple beats from multiple transactions 
// into a single buffer. used during DMA reads into scratchpad
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import Util._

class FgDMAWriteResponder[T <: Data]
  (config: GemminiArrayConfig[T], max_bytes: Int)
  (implicit p: Parameters) extends Module {
  import config._

  // I/O interface
  val io = IO(new Bundle {
    val peek = new FgXactTrackerPeekIO(config, max_bytes)
    val tl_d = Flipped(Decouple(new Bundle {
      val xactid = UInt(LOG2_MAX_DMA_REQS.W)
      val last = Bool()
    }))
    val pop  = Output(Valid(UInt(LOG2_MAX_DMA_REQS.W)))
    val resp = Decoupled(new FgStreamWriteResponse)
  })

  // next-state logic
  val total_bytes = io.peek.entry.total_bytes
  val txn_bytes   = io.peek.entry.txn_bytes // useful bytes in this txn
  val rob_id      = io.peek.entry.rob_id

  val bytes_completed      = RegInit(0.U(LOG2_MAX_TRANSFER_BYTES.W))
  val bytes_completed_next = bytes_completed + txn_bytes
  val finished_all_bytes   = (bytes_completed_next === total_bytes)

  val last_xactid    = RegInit(0.U(LOG2_MAX_DMA_REQS.W))
  val changed_xactid = last_xactid =/= io.tl_d.xactid

  // output-logic
  io.peek.xactid := io.tl_d.xactid
  io.tl_d.ready  := false.B
  io.pop.valid   := false.B
  io.pop.bits    := rob_id
  io.resp.valid  := false.B
  io.resp.rob_id := rob_id
        
  //---------------------------------
  // FSM
  //---------------------------------
  val (s_IDLE :: s_ACCUMULATE :: s_WAIT_FOR_RESP :: Nil) = Enum(3)
  val state = RegInit(s_IDLE)

  def process_txn(first_txn: Boolean) = {
    assert(io.tl_d.bits.last, "tilelink write txn result not 'last'")
    last_xactid := io.tl_d.bits.xactid
    bytes_completed := bytes_completed_next
    state := s_ACCUMULATE
    when (finished_all_bytes) {
      assert((first_txn.B || !changed_xactid), 
        "xactid changed before all xacts in a DMA transfer finished!")
      io.resp.valid := true.B
      state := s_WAIT_FOR_RESP
      when (io.resp.fire()) {
        io.pop.valid := true.B
        bytes_completed := 0.U
        state := s_IDLE
      }
    }
  }

  switch (state) {
    is (s_IDLE) {
      bytes_completed := 0.U
      io.tl_d.ready := true.B
      when (io.tl_d.fire()) {
        process_txn(true)
      }
    }
    is (s_ACCUMULATE) {
      io.tl_d.ready := true.B
      when (io.tl_d.fire()) {
        process_txn(false)
      }
    }
    is (s_WAIT_FOR_RESP) {
      io.resp.valid := true.B
      when (io.resp.fire()) {
        io.pop.valid := true.B
        bytes_completed := 0.U
        state := s_IDLE
      }
    }
  }
}

class FgBeatMerger[T <: Data](config: GemminiArrayConfig[T], max_bytes: Int)
  (implicit p: Parameters) extends Module {
  import config._

  // I/O interface
  val io = IO(new Bundle {
    val peek  = new FgXactTrackerPeekIO(config)
    val pop   = Output(Valid(UInt(LOG2_MAX_DMA_REQS.W)))
    val beat  = Flipped(Decoupled(FgStreamBeatData(config)))
    val resp  = Decoupled(new FgStreamReadResponse(max_bytes))
  })

  // internal logic/state
  val buffer = RegInit(0.U((max_bytes*8).W))
  val lrange = RegInit(0.U.asTypeOf[FgLocalRange])
  val rob_id = RegInit(UInt(LOG2_ROB_ENTRIES.W))

  io.pop.valid   := false.B
  io.pop.xactid  := io.beat.bits.xactid
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
        when (io.beat.bits.is_last_beat && io.peek.entry.is_last_txn) {
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

//============================================================================
// FgTxnSplitter
// - splits a write request into transactions and transactions into TL-A beats
//============================================================================
class FgTxnSplitter[T <: Data](config: GemminiArrayConfig[T], max_bytes: Int)
  (implicit p: Parameters) extends Module {
  import config._
  //---------------------------------
  // I/O interface
  //---------------------------------
  val io = IO(new Bundle {
    // req is driven from FgDMA req port
    val req = Flipped(Decoupled(new FgStreamWriteRequest(max_bytes)))
    // txn is driven from the FgControl
    val txn = Flipped(Decoupled(new FgTxnSplitterTxn(config, max_bytes)))
    // combinationally view the FgXactTracker entry for txn's xactid
    val peek = new FgXactTrackerPeekIO(config, max_bytes)
    // output beats to tl_a interface
    val tl_a = Decoupled(new Bundle {
      val xactid    = UInt(LOG2_MAX_DMA_REQS.W)
      val paddr     = UInt(coreMaxAddrBits.W)
      val log2_size = UInt(log2Up(log2Up(max_bytes+1)).W)
      val is_full   = Bool()
      val data      = Output(UInt((DMA_BUS_BYTES*8).W))
      val mask      = Output(UInt(DMA_BUS_BYTES.W))
    })
  })

  //---------------------------------
  // internal state managed by FSM
  //---------------------------------
  // pre-load the req, which comes in 1+ cycles before the 1st matching txn
  val req = Queue(io.req, 2)
  val cur_req = Reg(new FgStreamWriteRequest(max_bytes))
  val data = cur_req.bits.data
  
  // current txn that we are splitting into beats
  val txn = Queue(io.txn, 4)
  val cur_txn = Reg(new TxnSplitterTxn(config, max_bytes))
  io.peek.xactid := cur_txn.xactid

  val useful_bytes   = io.peek.entry.useful_bytes
  val data_start_idx = io.peak.entry.data_start_idx
  val data_end_idx   = data_start_idx + useful_bytes - 1.U
  val txn_bytes      = io.peek.entry.txn_bytes
  val txn_log2_bytes = io.peek.entry.txn_log2_bytes
  val txn_start_idx  = io.peek.entry.txn_start_idx
  val paddr          = io.peek.entry.paddr       
 
  val useful_bytes_sent = RegInit(0.U(LOG2_MAX_TRANSFER_BYTES.W))
  val txn_bytes_sent    = RegInit(0.U(LOG2_MAX_DMA_BYTES.W))
  val beat_idx          = RegInit(0.U(LOG2_MAX_DMA_BEATS.W))

  //---------------------------------
  // output logic
  //---------------------------------
  // calculate beat offsets into data-buffer
  val beat_start_idx = txn_start_idx + (beat_idx * DMA_BUS_BYTES)
  val beat_end_idx   = beat_start_idx + DMA_BUS_BYTES - 1.U
  val beat_data_full = Mux(beat_start_idx > data_start_idx,
                           data >> ((beat_start_idx - data_start_idx)*8.U),
                           data << ((data_start_idx - beat_start_idx)*8.U))
  val beat_data      = beat_data_full(DMA_BEAT_BITS-1, 0)

  // beat write-mask calculations
  val is_full        = (beat_start_idx >= data_start_idx) &&
                       (beat_end_idx   <= data_end_idx)
  val is_empty       = (beat_start_idx  > data_end_idx) ||
                       (beat_end_idx    < data_start_idx)
  val start_mask_offset = Mux(is_full || is_empty, 0.U, 
                              Mux(beat_start_idx >= data_start_idx, 0.U,
                                  data_start_idx - beat_start_idx))
  val end_mask_offset   = Mux(is_full || is_empty, 0.U, 
                              Mux(beat_end_idx <= data_end_idx, 0.U,
                                  beat_start_idx - data_start_idx))
  val beat_mask = Mux(is_full || is_empty, 0.U,
                    (((1.U << (DMA_BEAT_BYTES1.U - end_mask_offset)) - 1.U)
                      << start_mask_offset))
 
  //---------------------------------
  // output-logic
  //---------------------------------
  io.tl_a.valid          := false.B
  io.tl_a.bits.is_full   := is_full
  io.tl_a.bits.xactid    := txn.xactid
  io.tl_a.bits.paddr     := paddr
  io.tl_a.bits.log2_size := log2_size
  io.tl_a.bits.data      := beat_data
  io.tl_a.bits.mask      := beat_mask

  //---------------------------------
  // next-state logic
  //---------------------------------
  val useful_beat_bytes = DMA_BEAT_BYTES - start_mask_offset - end_mask_offset
  val useful_bytes_sent_next = useful_bytes_sent + useful_beat_bytes
  val txn_bytes_sent_next = txn_bytes_sent + DMA_BEAT_BYTES
  val xfer_finished_next = (useful_bytes_sent_next === useful_bytes)
  val txn_finished_next = (txn_bytes_sent_next === txn_bytes)

  //---------------------------------
  // FSM
  //---------------------------------
  val (s_PENDING_REQ :: s_PENDING_TXN :: s_SEND_BEAT :: Nil) = Enum(3)
  val state = RegInit(s_PENDING_REQ)

  def start_next_txn(dummy: Int = 0) = {
    txn_bytes_sent := 0.U
    beat_idx := 0.U
    txn.ready := true.B
    when (txn.fire()) {
      cur_txn := txn.bits
      state := s_SEND_BEAT
    } .otherwise {
      state := s_PENDING_TXN
    }
  }

  def start_next_req(dummy: Int = 0) = {
    useful_bytes_sent := 0.U
    req.ready := true.B
    when (req.fire()) {
      cur_req := req.bits
      start_next_txn()
    } .otherwise {
      state := s_PENDING_REQ
    }
  }

  switch (state) {
    is (s_PENDING_REQ) {
      start_next_req()
    }
    is (s_PENDING_TXN) {
      start_next_txn()
    }
    is (s_SEND_BEAT) {
      io.tl_a.valid := true.B
      when(io.tl_a.fire()) {
        useful_bytes_sent := useful_bytes_sent_next
        txn_bytes_sent := txn_bytes_sent_next
        beat_idx := beat_idx + 1.U
        when (xfer_finished_next) {
          assert(txn_finished_next, "req finished but one of its txn did not")
          start_next_req()
        } .elsewhen (txn_finished_next) {
          start_next_txn()
        }
      }
    }
  }
}
