//===========================================================================
// BeatMerger
// - shifts/aligns and merges multiple beats from multiple transactions 
// into a single buffer. used during DMA reads into scratchpad
// - achieves max throughput of 1 resp/cycle
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import Util._

class FgDMAStoreResponder[T <: Data]
  (config: FgGemminiArrayConfig[T], max_xfer_bytes: Int)
  (implicit p: Parameters) extends Module {
  import config._

  // I/O interface
  val io = IO(new Bundle {
    val tl_d = Flipped(Decoupled(new Bundle {
      val xactid = UInt(DMA_REQS_IDX.W)
    }))
    val peek = new FgDMATrackerPeekIO(config, max_xfer_bytes)
    val pop  = Output(Valid(UInt(DMA_REQS_IDX.W)))
    val resp = Decoupled(new FgDMAStoreResponse(config))
    val busy = Output(Bool())
  })

  // next-state logic
  val total_useful_bytes = io.peek.entry.req_useful_bytes
  val txn_useful_bytes   = io.peek.entry.txn_useful_bytes
  val rob_id             = io.peek.entry.rob_id

  val useful_bytes_completed      = RegInit(0.U(log2Ceil(max_xfer_bytes+1).W))
  val useful_bytes_completed_next = useful_bytes_completed + txn_useful_bytes
  val finished_all_bytes          = (total_useful_bytes > 0.U) &&
                                    (useful_bytes_completed_next === 
                                     total_useful_bytes)

  val last_xactid    = RegInit(0.U(DMA_REQS_IDX.W))
  val changed_xactid = last_xactid =/= io.tl_d.bits.xactid

  // output-logic
  io.peek.xactid      := io.tl_d.bits.xactid
  io.tl_d.ready       := false.B
  io.pop.valid        := false.B
  io.pop.bits         := io.tl_d.bits.xactid
  io.resp.valid       := false.B
  io.resp.bits.rob_id := rob_id
        
  //---------------------------------
  // FSM
  //---------------------------------
  val (s_IDLE :: s_ACCUMULATE :: s_WAIT_FOR_RESP :: Nil) = Enum(3)
  val state = RegInit(s_IDLE)

  def process_txn(first_txn: Boolean) = {
    last_xactid := io.tl_d.bits.xactid
    useful_bytes_completed := useful_bytes_completed_next
    state := s_ACCUMULATE
    when (finished_all_bytes) {
      assert((first_txn.B || !changed_xactid), 
        "xactid changed before all xacts in a DMA transfer finished!")
      io.resp.valid := true.B
      state := s_WAIT_FOR_RESP
      when (io.resp.fire()) {
        io.pop.valid := true.B
        useful_bytes_completed := 0.U
        state := s_IDLE
      }
    }
  }

  switch (state) {
    is (s_IDLE) {
      useful_bytes_completed := 0.U
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
        useful_bytes_completed := 0.U
        state := s_IDLE
      }
    }
  }

  //-----------------------------------------------
  // busy tracker
  //-----------------------------------------------
  io.busy := (state =/= s_IDLE)
}

