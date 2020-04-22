package gemmini

import chisel3._
import chisel3.util._
import gemmini.Util.UDValid

//===========================================================================
// tracked outstanding entry
//===========================================================================
class FgXactTrackerEntry[T <: Data]
  (config: GemminiArrayConfig[T], max_bytes: Int)(implicit p: Parameters) 
  extends CoreBundle {
  import config._
  val lrange       = new FgLocalRange
  val rob_id       = UInt(LOG2_ROB_ENTRIES.W)
  val start_rshift = UInt(LOG2_MAX_DMA_BYTES.W)
  val txn_lshift   = UInt(log2Up(max_bytes).W)
  val is_last_txn  = Bool()
}

//===========================================================================
// XactTracker Interfaces
//===========================================================================
class FgXactTrackerPeekIO[T <: Data]
  (config: GemminiArrayConfig[T], max_bytes: Int)(implicit p: Parameters) 
  extends CoreBundle {
  import config._
  val xactid = Output(UInt(LOG2_MAX_DMA_REQS.W))
  val pop    = Output(Bool())
  val entry  = Input(new FgXactTrackerEntry(config, max_bytes))
}

//===========================================================================
// XactTracker 
// - track outstanding DMA requests with their offset/length
// - all outstanding DMA reqs are merged to/split from a single acc/sp row!
//===========================================================================
class FgXactTracker[T <: Data](config: GemminiArrayConfig[T], max_bytes: Int)
  (implicit p: Parameters) extends Module {
  import config._

  val io = IO(new Bundle {
    val next_xactid = Output(UInt(LOG2_MAX_DMA_REQS.W))
    val alloc = Flipped(Decoupled(new FgXactTrackerEntry(config, max_bytes)))
    val peek  = Flipped(new FgXactTrackerPeekIO(config, max_bytes))
    val busy  = Output(Bool())
  })

  // outstanding request registers
  val entries = Reg(Vec(MAX_DMA_REQS, 
                    UDValid(new FgXactTrackerEntry(config))))
  when (reset.toBool()) {
    entries.foreach(_.valid := false.B)
  }
  io.busy := entries.map(_.valid).reduce(_ || _)

  // interface to stream reader/writer
  val free_entry = MuxCase((MAX_DMA_REQS-1).U, entries.zipWithIndex.map { 
    case (e, i) => !e.valid -> i.U 
  })
  io.next_xactid := free_entry
  io.alloc.ready := !entries.map(_.valid).reduce(_ && _)
  when (io.alloc.fire()) {
    entries(free_entry).valid := true.B
    entries(free_entry).bits := io.alloc.bits
  }

  // interface to beat-merger
  io.peek.entry := entries(io.peek.xactid).bits
  when (io.peek.pop) {
    assert(entries(io.peek.xactid).valid)
    entries(io.peek.xactid).valid := false.B
  }
}
