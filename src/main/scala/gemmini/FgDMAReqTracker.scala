package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import gemmini.Util._

//===========================================================================
// FgDMAReqTracker Interfaces
//===========================================================================
class FgDMAReqIncr[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val all_txns_sent = Bool()
  val rob_id = Valid(ROB_ENTRIES_IDX.W)
}

//===========================================================================
// FgDMAReqTracker 
//===========================================================================
class FgDMAReqTracker[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends Module {
  import config._
  //-------------------------------------------
  // I/O interface
  //-------------------------------------------
  val io = IO(new Bundle {
    val alloc = Flipped(Valid(ROB_ENTRIES_IDX.W))
    val incr = Flipped(Valid(new FgDMAReqIncr(config)))
    val decr = Flipped(Decoupled(ROB_ENTRIES_IDX.W))
    val resp = Decoupled(new FgDMAResponse(config))
    val busy = Output(Bool())
  })

  //-------------------------------------------
  // Internal State
  //-------------------------------------------
  class FgDMAReqTrackerEntry[T <: Data](val config: FgGemminiArrayConfig[T])
    (implicit p: Parameters) extends CoreBundle {
    import config._
    val valid         = Bool()
    val all_txns_sent = Bool()
    val txn_count     = UInt(DMA_REQ_TXNS_CTR.W)
  }
  val entries = Reg(Vec(ROB_ENTRIES, new FgDMAReqTrackerEntry(config)))
  when (reset.toBool()) {
    entries.foreach(_.valid := false.B)
  }

  //-------------------------------------------
  // default I/O
  //-------------------------------------------
  io.busy             := entries.map(_.valid).reduce(_ || _)
  io.decr.ready       := false.B
  io.resp.valid       := false.B
  io.resp.bits.rob_id := entries(io.decr.bits)

  //-------------------------------------------
  // the first txn was sent off for this dma request
  //-------------------------------------------
  when (io.alloc.fire()) {
    val entry = entries(io.alloc.bits)
    assert(!entry.valid, "allocating an existing txn")

    entry.valid := true.B
    entry.all_txns_sent := false.B
    entry.txn_count := 0.U
  }

  //-------------------------------------------
  // another txn was sent off for this dma request
  //-------------------------------------------
  when (io.incr.fire()) {
    val entry = entries(io.incr.bits.rob_id)
    assert(entry.valid, "incrementing an existing txn")
    assert(!entry.all_txns_sent, "incrementing a txn with all_txns_sent")

    entry.valid := true.B
    entry.all_txns_sent := io.incr.bits.all_txns_sent
    entry.txn_count := entry.txn_count + 1.U
  }

  //-------------------------------------------
  // decrement outstanding txns for this req, and when all txns are finished,
  // send a MemOpResponse back to the MemOpController
  //-------------------------------------------
  val decr_entry   = entries(io.decr.bits)
  val is_last_decr = decr_entry.txn_count === 1.U && decr_entry.all_txns_sent

  io.decr.ready := (io.resp.ready || !is_last_decr) &&
                   (!io.incr.fire()|| (io.incr.bits.rob_id =/= io.decr.bits))

  when (io.decr.fire()) {
    assert(decr_entry.valid, "incrementing an existing txn")
    assert(decr_entry.txn_count > 0.U, "decrement when txn_count === 0")

    decr_entry.txn_count := decr_entry.txn_count - 1.U
    when (is_last_decr) {
      decr_entry.valid := false.B
      io.resp.valid := true.B
      assert(io.resp.fire())
    }
  }
}
