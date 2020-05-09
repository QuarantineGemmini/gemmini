package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import gemmini.Util._

//===========================================================================
// outstanding requests (have as many as outstanding txns)
//===========================================================================
class FgDMAReqTrackerEntry[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val rob_id        = UInt(ROB_ENTRIES_IDX.W)
  val all_txns_sent = Bool()
  val txn_count     = UInt(DMA_REQ_TXNS_CTR.W)
}

//===========================================================================
// FgDMAReqTracker Interfaces
//===========================================================================
class FgDMAReqIncr[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val reqid = UInt(DMA_REQS_IDX.W)
  val all_txns_sent = Bool()
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
    val alloc = Flipped(Decoupled(UInt(ROB_ENTRIES_IDX.W)))
    val curid = Output(UInt(DMA_REQS_IDX.W))
    val incr  = Flipped(Valid(new FgDMAReqIncr(config)))
    val decr  = Flipped(Decoupled(UInt(DMA_REQS_IDX.W)))
    val resp  = Decoupled(new FgDMAResponse(config))
    val busy  = Output(Bool())
  })

  //----------------------------------
  // outstanding transaction registers
  //----------------------------------
  val entries = Reg(Vec(DMA_REQS, UDValid(new FgDMAReqTrackerEntry(config))))
  when (reset.toBool()) {
    entries.foreach(_.valid := false.B)
  }
  val last_allocated = RegInit(0.U(DMA_REQS_IDX.W))
  io.curid := last_allocated
  io.busy := entries.map(_.valid).reduce(_ || _)

  //----------------------------------
  // interface to allocator
  //----------------------------------
  val free_entry = MuxCase((DMA_REQS-1).U, entries.zipWithIndex.map {
    case (e, i) => !e.valid -> i.U
  })
  io.alloc.ready := !entries.map(_.valid).reduce(_ && _)
  when (io.alloc.fire()) {
    last_allocated := free_entry

    entries(free_entry).valid              := true.B
    entries(free_entry).bits.rob_id        := io.alloc.bits
    entries(free_entry).bits.all_txns_sent := false.B
    entries(free_entry).bits.txn_count     := 0.U
  }

  //-------------------------------------------
  // another txn was sent off for this dma request
  //-------------------------------------------
  when (io.incr.fire()) {
    val entry = entries(io.incr.bits.reqid)
    assert(entry.valid, "incrementing an existing txn")
    assert(!entry.bits.all_txns_sent,"incrementing a txn with all_txns_sent")

    entry.bits.all_txns_sent := io.incr.bits.all_txns_sent
    entry.bits.txn_count := entry.bits.txn_count + 1.U
  }

  //-------------------------------------------
  // decrement outstanding txns for this req, and when all txns are finished,
  // send a MemOpResponse back to the MemOpController
  //-------------------------------------------
  val decr_entry   = entries(io.decr.bits)
  val is_last_decr = (decr_entry.bits.txn_count === 1.U) && 
                      decr_entry.bits.all_txns_sent

  io.decr.ready := (io.resp.ready || !is_last_decr) &&
                   (!io.incr.fire() || (io.incr.bits.reqid =/= io.decr.bits))
  io.resp.valid := false.B
  io.resp.bits.rob_id := decr_entry.bits.rob_id

  when (io.decr.fire()) {
    assert(decr_entry.valid, "incrementing an existing txn")
    assert(decr_entry.bits.txn_count > 0.U, "decrement when txn_count === 0")

    decr_entry.bits.txn_count := decr_entry.bits.txn_count - 1.U
    when (is_last_decr) {
      decr_entry.valid := false.B
      io.resp.valid := true.B
      assert(io.resp.fire())
    }
  }
}
