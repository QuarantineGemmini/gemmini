//===========================================================================
// BeatMerger
// - shifts/aligns and merges multiple beats from multiple transactions 
// into a single buffer. used during DMA reads into scratchpad
// - achieves max throughput of 1 resp/cycle
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
    val tl_d = Flipped(Decouple(new Bundle {
      val xactid = UInt(LOG2_MAX_DMA_REQS.W)
    }))
    val peek = new FgXactTrackerPeekIO(config, max_bytes)
    val pop  = Output(Valid(UInt(LOG2_MAX_DMA_REQS.W)))
    val resp = Decoupled(new FgStreamWriteResponse)
    val busy = Output(Bool())
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

  //-----------------------------------------------
  // busy tracker
  //-----------------------------------------------
  io.busy := (state =/= s_IDLE)
}

