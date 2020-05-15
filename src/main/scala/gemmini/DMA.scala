package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.tile._
import testchipip.TLHelper

import Util._

class DMAResponse[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

//===========================================================================
// DMA read interface (dram -> scratchpad)
//===========================================================================
class DMALoadRequest[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new LocalRange(config)
  val status = new MStatus
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

class DMALoadDataChunk[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val is_acc = Bool()
  val row    = UInt((ACC_ROWS_IDX max SP_ROWS_IDX).W)
  val mask   = UInt(DIM.W)
  val data   = UInt(ACC_ROW_BITS.W)
}

class DMALoad[T <: Data](
  config: GemminiArrayConfig[T], name: String = "dma-load")
  (implicit p: Parameters) extends LazyModule {
  import config._

  val node = TLHelper.makeClientNode(name, IdRange(0, DMA_REQS))

  lazy val module = new LazyModuleImp(this)
    with HasCoreParameters with MemoryOpConstants {
    // I/O interface
    val (tl, edge) = node.out(0)

    val io = IO(new Bundle {
      val req   = Flipped(Decoupled(new DMALoadRequest(config)))
      val chunk = Decoupled(new DMALoadDataChunk(config))
      val resp  = Decoupled(new DMAResponse(config))
      val tlb   = new FrontendTLBIO
      val busy  = Output(Bool())
      val flush = Input(Bool())
    })

    val control_ready = WireDefault(false.B)
    val req_tracker_ready = WireDefault(false.B)
    val req = Wire(Flipped(Decoupled(new DMAControlRequest(config))))
    io.req.ready    := control_ready && req_tracker_ready
    req.ready       := control_ready && req_tracker_ready
    req.valid       := io.req.valid
    req.bits.vaddr  := io.req.bits.vaddr
    req.bits.lrange := io.req.bits.lrange
    req.bits.status := io.req.bits.status

    //-----------------------------------------------
    // track outstanding transactions for each operation
    //-----------------------------------------------
    val tracker = Module(new DMATracker(config, 1))

    val req_tracker = Module(new DMAReqTracker(config))
    req_tracker_ready          := req_tracker.io.alloc.ready
    req_tracker.io.alloc.valid := io.req.fire()
    req_tracker.io.alloc.bits  := io.req.bits.rob_id
    io.resp                    <> req_tracker.io.resp
    io.busy                    := req_tracker.io.busy

    //-----------------------------------------------
    // tlb translations and tilelink txn dispatch
    //-----------------------------------------------
    val control = Module(new DMAControl(config, true))
    control_ready          := control.io.req.ready
    control.io.req.valid   := req.fire()
    control.io.req.bits    := req.bits
    control.io.txn_nextid  := tracker.io.nextid
    tracker.io.alloc       <> control.io.alloc
    control.io.req_curid   := req_tracker.io.curid
    req_tracker.io.incr    := control.io.incr
    io.tlb                 <> control.io.tlb
    control.io.flush       := io.flush

    //-----------------------------------------------
    // merge response beats into buffer
    //-----------------------------------------------
    val merger = Module(new DMABeatMerger(config))
    tracker.io.peek(0)  <> merger.io.peek
    tracker.io.pop      <> merger.io.pop
    req_tracker.io.decr <> merger.io.decr
    io.chunk            <> merger.io.chunk

    //-----------------------------------------------
    // tile-link A-channel request
    //-----------------------------------------------
    tl.a.valid := control.io.dispatch.valid
    control.io.dispatch.ready := tl.a.ready
    tl.a.bits := edge.Get(
      fromSource = control.io.dispatch.bits.xactid,
      toAddress  = control.io.dispatch.bits.paddr,
      lgSize     = control.io.dispatch.bits.txn_log2_bytes,
    )._2

    //-----------------------------------------------
    // tile-link D-channel response (systolic)
    //-----------------------------------------------
    tl.d.ready                       := merger.io.beat.ready
    merger.io.beat.valid             := tl.d.valid
    merger.io.beat.bits.xactid       := tl.d.bits.source
    merger.io.beat.bits.data         := tl.d.bits.data
    merger.io.beat.bits.beat_idx     := edge.count(tl.d)._4 
    merger.io.beat.bits.is_last_beat := edge.last(tl.d)
  }
}

//===========================================================================
// DMA write interface (scratchpad -> dram)
//===========================================================================
class DMAStoreRequest[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val vaddr  = UInt(coreMaxAddrBits.W)
  val data   = UInt(SP_ROW_BITS.W)
  val cols   = UInt(DIM_CTR.W) // don't support col2im output!
  val status = new MStatus
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

class DMAStore[T <: Data](
  config: GemminiArrayConfig[T], name: String = "dma-store")
  (implicit p: Parameters) extends LazyModule {
  import config._

  val node = TLHelper.makeClientNode(name, IdRange(0, DMA_REQS))

  lazy val module = new LazyModuleImp(this) 
    with HasCoreParameters {
    // I/O interface
    val (tl, edge) = node.out(0)
    
    val io = IO(new Bundle {
      val req   = Flipped(Decoupled(new DMAStoreRequest(config)))
      val resp  = Decoupled(new DMAResponse(config))
      val tlb   = new FrontendTLBIO
      val busy  = Output(Bool())
      val flush = Input(Bool())
    })

    val control_ready = WireDefault(false.B)
    val splitter_ready = WireDefault(false.B)
    val req_tracker_ready = WireDefault(false.B)
    val req = Wire(Flipped(Decoupled(new DMAControlRequest(config))))
    io.req.ready    := control_ready && splitter_ready && req_tracker_ready 
    req.ready       := control_ready && splitter_ready && req_tracker_ready
    req.valid       := io.req.valid
    req.bits.vaddr  := io.req.bits.vaddr
    req.bits.lrange := io.req.bits.lrange
    req.bits.status := io.req.bits.status

    //-----------------------------------------------
    // track outstanding transactions for each operation
    //-----------------------------------------------
    val tracker = Module(new DMATracker(config, 2))

    val req_tracker = Module(new DMAReqTracker(config))
    req_tracker_ready          := req_tracker.io.alloc.ready
    req_tracker.io.alloc.valid := io.req.fire()
    req_tracker.io.alloc.bits  := io.req.bits.rob_id
    io.resp                    <> req_tracker.io.resp
    io.busy                    := req_tracker.io.busy

    //-----------------------------------------------
    // tlb translations and tilelink txn dispatch
    //-----------------------------------------------
    val control = Module(new DMAControl(config, false))
    control_ready         := control.io.req.ready
    control.io.req.valid  := req.fire()
    control.io.req.bits   := req.bits
    control.io.txn_nextid := tracker.io.nextid
    tracker.io.alloc      <> control.io.alloc
    control.io.req_curid  := req_tracker.io.curid
    req_tracker.io.incr   := control.io.incr
    io.tlb                <> control.io.tlb
    control.io.flush      := io.flush

    //-----------------------------------------------
    // track outstanding requests
    //-----------------------------------------------
    val splitter = Module(new DMASplitter(config))
    splitter_ready        := splitter.io.req.ready
    splitter.io.req.valid := io.req.fire()
    splitter.io.req.bits  := io.req.bits
    splitter.io.txn       <> control.io.dispatch
    tracker.io.peek(0)    <> splitter.io.peek

    //-----------------------------------------------
    // tile-link A-channel request
    //-----------------------------------------------
    splitter.io.tl_a.ready := tl.a.ready
    tl.a.valid := splitter.io.tl_a.valid
    tl.a.bits := Mux(splitter.io.tl_a.bits.is_full, 
      edge.Put(
        fromSource = splitter.io.tl_a.bits.xactid,
        toAddress  = splitter.io.tl_a.bits.paddr,
        lgSize     = splitter.io.tl_a.bits.log2_bytes,
        data       = splitter.io.tl_a.bits.data.asUInt(),
      )._2,
      edge.Put(
        fromSource = splitter.io.tl_a.bits.xactid,
        toAddress  = splitter.io.tl_a.bits.paddr,
        lgSize     = splitter.io.tl_a.bits.log2_bytes,
        data       = splitter.io.tl_a.bits.data.asUInt(),
        mask       = splitter.io.tl_a.bits.mask.asUInt(),
      )._2)

    //-----------------------------------------------
    // tile-link D-channel response
    //-----------------------------------------------
    tracker.io.peek(1).xactid := tl.d.bits.source

    tl.d.ready                := req_tracker.io.decr.ready
    req_tracker.io.decr.valid := tl.d.valid
    req_tracker.io.decr.bits  := tracker.io.peek(1).entry.reqid

    tracker.io.pop.valid := tl.d.fire()
    tracker.io.pop.bits  := tl.d.bits.source
  }
}
