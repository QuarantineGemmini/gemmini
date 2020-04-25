package gemmini

import chisel3._
import chisel3.util._

import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import testchipip.TLHelper
import freechips.rocketchip.rocket._

import Util._

//===========================================================================
// DMA read interface (dram -> scratchpad)
// - max_bytes: max bytes written to scratchpad in an operation
//===========================================================================
class FgStreamReadRequest[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange(config)
  val status = new MStatus
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

class FgStreamReadResponse[T <: Data]
  (config: GemminiArrayConfig[T], max_bytes: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val data   = UInt((max_bytes*8).W)
  val lrange = new FgLocalRange(config)
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

class FgDMAReader[T <: Data](config: GemminiArrayConfig[T], 
  name: String = "stream-reader", max_bytes: Int)
  (implicit p: Parameters) extends LazyModule {
  import config._

  val node = TLHelper.makeClientNode(name, IdRange(0, MAX_DMA_REQS))

  lazy val module = new LazyModuleImp(this)
    with HasCoreParameters with MemoryOpConstants {
    // I/O interface
    val (tl, edge) = node.out(0)

    val io = IO(new Bundle {
      val req   = Flipped(Decoupled(new FgStreamReadRequest(config)))
      val resp  = Decoupled(new FgDMAReadResponse(config, max_bytes))
      val tlb   = new FrontendTLBIO
      val busy  = Output(Bool())
      val flush = Input(Bool())
    })

    val req = Wire(Flipped(Decoupled(new FgDMAControlRequest(config))))
    io.req.ready    := req.ready
    req.valid       := io.req.valid
    req.bits.vaddr  := io.req.bits.vaddr
    req.bits.lrange := io.req.bits.lrange
    req.bits.status := io.req.bits.status
    req.bits.rob_id := io.req.bits.rob_id

    //-----------------------------------------------
    // track outstanding transactions for each operation
    //-----------------------------------------------
    val tracker = Module(new FgDMATracker(config, max_bytes, 2))

    //-----------------------------------------------
    // tlb translations and tilelink txn dispatch
    //-----------------------------------------------
    val control = Module(new FgDMAControl(config, max_bytes, true))
    control.io.req    <> req
    control.io.nextid := tracker.io.nextid
    tracker.io.alloc  <> control.io.alloc
    io.tlb            <> control.io.tlb
    control.io.flush  := io.flush

    //-----------------------------------------------
    // merge response beats into buffer
    //-----------------------------------------------
    val merger = Module(new FgDMABeatMerger(config, max_bytes))
    tracker.io.peek(0) <> merger.io.peek
    tracker.io.pop     <> merger.io.pop
    io.resp            <> merger.io.resp

    //-----------------------------------------------
    // tile-link A-channel request
    //-----------------------------------------------
    tl.a.valid := control.io.dispatch.valid
    control.io.dispatch.ready := tl.a.ready
    tracker.io.peek(1).xactid := control.io.dispatch.bits.xactid
    tl.a.bits := edge.Get(
      fromSource = tracker.io.peek(1).entry.xactid,
      toAddress  = tracker.io.peek(1).entry.paddr,
      lgSize     = tracker.io.peek(1).entry.txn_log2_size
    )._2

    //-----------------------------------------------
    // tile-link D-channel response (systolic)
    //-----------------------------------------------
    tl.d.ready                       := merger.io.beat.ready
    merger.io.beat.valid             := tl.d.valid
    merger.io.beat.bits.xactid       := tl.d.bits.source
    merger.io.beat.bits.data         := tl.d.bits.data
    merger.io.beat.bits.beat_idx     := edge.count(tl.d) 
    merger.io.beat.bits.is_last_beat := edge.last(tl.d)

    //-----------------------------------------------
    // busy signal
    //-----------------------------------------------
    io.busy := tracker.io.busy || control.io.busy || merger.io.busy
  }
}

//===========================================================================
// DMA write interface (scratchpad -> dram)
// - max_bytes: max bytes written to dram in an operation
//===========================================================================
class FgStreamWriteRequest[T <: Data]
  (config: GemminiArrayConfig[T], max_bytes: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val data   = UInt((max_bytes*8).W)
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange(config)
  val status = new MStatus
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

class FgStreamWriteResponse(implicit p: Parameters) extends CoreBundle {
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

class StreamWriter[T <: Data](config: GemminiArrayConfig[T], 
  name: String = "stream-writer", max_bytes: Int)
  (implicit p: Parameters) extends LazyModule {
  import config._

  val node = TLHelper.makeClientNode(name, IdRange(0, MAX_DMA_REQS))

  lazy val module = new LazyModuleImp(this) 
    with HasCoreParameters {
    // I/O interface
    val (tl, edge) = node.out(0)
    
    val io = IO(new Bundle {
      val req  = Flipped(Decoupled(new StreamWriteRequest(config,max_bytes)))
      val resp = Decoupled(new FgDMAWriteResponse)
      val tlb  = new FrontendTLBIO
      val busy = Output(Bool())
      val flush = Input(Bool())
    })

    val req = Wire(Flipped(Decoupled(new FgDMAControlRequest(config))))
    io.req.ready    := req.ready
    req.valid       := io.req.valid
    req.bits.vaddr  := io.req.bits.vaddr
    req.bits.lrange := io.req.bits.lrange
    req.bits.status := io.req.bits.status
    req.bits.rob_id := io.req.bits.rob_id

    //-----------------------------------------------
    // track outstanding requests
    //-----------------------------------------------
    val tracker = Module(new FgDMATracker(config, max_bytes, 2))

    //-----------------------------------------------
    // tlb translations and tilelink txn dispatch
    //-----------------------------------------------
    val control = Module(new FgDMAControl(config, max_bytes, false))
    control.io.req    <> req
    control.io.nextid := tracker.io.nextid
    tracker.io.alloc  <> control.io.alloc
    io.tlb            <> control.io.tlb
    control.io.flush  := io.flush

    //-----------------------------------------------
    // track outstanding requests
    //-----------------------------------------------
    val splitter = Module(new FgDMASplitter(config, max_bytes))
    assert(splitter.io.req.ready === true.B, "splitter blocked on req")
    splitter.io.req.valid := io.req.valid
    splitter.io.req.bits  := io.req.bits
    splitter.io.txn       <> control.io.dispatch
    tracker.io.peek(0)    <> splitter.io.txn

    //-----------------------------------------------
    // send response when all tilelink txns are complete
    //-----------------------------------------------
    val responder = Module(new FgDMAWriteResponder(config, max_bytes))
    tracker.io.peek(1) <> responder.io.peek
    tracker.io.pop     <> responder.io.pop
    io.resp            <> responder.io.resp

    //-----------------------------------------------
    // tile-link A-channel request
    //-----------------------------------------------
    splitter.io.tl_a.ready := tl.a.ready
    tl.a.valid := splitter.io.tl_a.valid
    tl.a.bits := Mux(splitter.io.tl_a.bits.is_full, 
      edge.Put(
        fromSource = splitter.io.tl_a.bits.xactid,
        toAddress  = splitter.io.tl_a.bits.paddr,
        lgSize     = splitter.io.tl_a.bits.log2_size,
        data       = splitter.io.tl_a.bits.data.asUInt(),
      )._2,
      edge.Put(
        fromSource = splitter.io.tl_a.bits.xactid,
        toAddress  = splitter.io.tl_a.bits.paddr,
        lgSize     = splitter.io.tl_a.bits.log2_size,
        data       = splitter.io.tl_a.bits.data.asUInt(),
        mask       = splitter.io.tl_a.bits.mask.asUInt(),
      )._2)

    //-----------------------------------------------
    // tile-link D-channel response
    //-----------------------------------------------
    responder.io.tl_d.valid := tl.d.valid
    tl.d.ready := responder.io.tl_d.valid
    responder.io.tl_d.bits.xactid := tl.d.bits.source

    //-----------------------------------------------
    // busy tracker
    //-----------------------------------------------
    io.busy := control.io.busy || tracker.io.busy || reponder.io.busy
  }
}
