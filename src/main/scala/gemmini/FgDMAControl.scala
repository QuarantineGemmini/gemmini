package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.constants._
import Util._

//===========================================================================
// request to FgDMAControl to start dma write/read operation
//===========================================================================
class FgDMAControlRequest[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange(config)
  val status = new MStatus
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

//===========================================================================
// request from FgDMAControl to the circuit that sends tilelink-A reqs
//===========================================================================
class FgDMADispatch[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val xactid = UInt(LOG2_MAX_DMA_REQS.W)
}

//===========================================================================
// FgDMAControl is same controller for DMA reads/writes
// - it handles translating vaddrs to paddrs
// - it handles splitting operations into tilelink transactions
// - it allocates entries in the FgDMATracker for each tilelink txn initiated
// - achieves max throughput of 1 dispatch/cycle
//===========================================================================
class FgDMAControl
  (config: GemminiArrayConfig[T], max_bytes: Int, is_read_mode: Boolean)
  (implicit p: Parameters) extends CoreModule with MemoryOpConstants {
  import config._
  //-----------------------------------------------
  // I/O interface
  //-----------------------------------------------
  val io = IO(new Bundle {
    val req      = Flipped(Decoupled(new FgDMAControlRequest(config)))
    val nextid   = Input(UInt(LOG2_MAX_DMA_REQS.W))
    val alloc    = Decoupled(new FgTrackerEntry(config, max_bytes))
    val tlb      = new FrontendTLBIO
    val flush    = Input(Bool())
    val dispatch = Decoupled(new FgDMADispatch(config))
    val busy     = Output(Bool())
  })

  //-----------------------------------------------
  // active request
  //-----------------------------------------------
  val req = Reg(new FgDMAControlRequest(config))
  val mstatus   = req.status
  val rob_id    = req.rob_id
  val lrange    = req.lrange
  val item_rows = lrange.rows
  val item_cols = lrange.cols
  val is_acc    = lrange.is_acc

  // initialized on io.req.fire()
  val total_useful_bytes     = RegInit(0.U(LOG2_MAX_TRANSFER_BYTES.W))
  val useful_bytes_left      = RegInit(0.U(LOG2_MAX_TRANSFER_BYTES.W))
  val total_bytes_requested  = RegInit(0.U((LOG2_MAX_TRANSFER_BYTES+1).W))
  val useful_bytes_requested = total_useful_bytes - useful_bytes_left

  //-----------------------------------------------
  // TLB Address translation
  //-----------------------------------------------
  val cur_vaddr     = RegInit(0.U(coreMaxAddrBits.W))
  val cur_vpn       = cur_vaddr(coreMaxAddrBits-1, pgIdxBits)
  val cur_ppn       = RegInit(0.U((coreMaxAddrBits - pgIdxBits).W))
  val cur_ppn_valid = withReset(io.flush.toBool()) { RegInit(false.B) }
  val cur_paddr     = Cat(cur_ppn, cur_vaddr(pgIdxBits-1, 0))

  io.tlb.req.valid                    := false.B
  io.tlb.req.bits.tlb_req.vaddr       := Cat(cur_vpn, 0.U(pgIdxBits.W))
  io.tlb.req.bits.tlb_req.passthrough := false.B
  io.tlb.req.bits.tlb_req.size        := 0.U
  io.tlb.req.bits.tlb_req.cmd         := if(is_read_mode) M_XRD else M_XWR
  io.tlb.req.bits.status              := mstatus

  //-----------------------------------------------
  // (combinational) Select the size and mask of the TileLink request
  //-----------------------------------------------
  class Txn extends Bundle {
    val bytes          = UInt(LOG2_MAX_DMA_BYTES).W)
    val log2_bytes     = UInt(log2Up(LOG2_MAX_DMA_BYTES).W)
    val useful_bytes   = UInt(LOG2_MAX_DMA_BYTES.W)
    val data_start_idx = UInt(LOG2_MAX_DMA_BYTES.W)
    val paddr          = UInt(paddrBits.W)
  }

  val candidate_txns = (DMA_BUS_BYTES to MAX_DMA_BYTES by DMA_BUS_BYTES)
                       .filter(bytes => isPow2(bytes)).map { bytes =>
    val log2_bytes    = log2Ceil(bytes)
    val paddr_aligned = Cat(cur_paddr(paddrBits-1, log2_bytes),
                            0.U(log2_bytes.W))
    val paddr_offset  = cur_paddr(log2_bytes-1, 0)

    val txn = Wire(new Txn())
    txn.useful_bytes   := minOf(bytes.U - paddr_offset, useful_bytes_left)
    txn.bytes          := bytes.U
    txn.log2_bytes     := log2_bytes.U
    txn.data_start_idx := paddr_offset
    txn.paddr          := paddr_aligned
    txn
  }
  val best_txn = candidate_txns.reduce { (best, cur) =>
    Mux(cur.useful_bytes > best.useful_bytes, cur, best)
  }
  val cur_useful_bytes   = best_txn.useful_bytes
  val cur_data_start_idx = best_txn.data_start_idx
  val cur_txn_bytes      = best_txn.bytes
  val cur_txn_log2_bytes = best_txn.log2_bytes
  val cur_paddr          = best_txn.paddr

  //-----------------------------------------------
  // allocate new tag for the tile-link request
  //-----------------------------------------------
  val start_rshift = RegInit(0.U(LOG2_MAX_TRANSFER_BYTES.W))
  val is_last_txn  = (cur_useful_bytes === useful_bytes_left)
  val is_first_txn = (useful_bytes_requested === 0.U)

  tracker.io.alloc.valid                 := false.B
  tracker.io.alloc.bits.lrange           := lrange
  tracker.io.alloc.bits.rob_id           := rob_id
  tracker.io.alloc.bits.req_useful_bytes := total_useful_bytes
  tracker.io.alloc.bits.data_start_idx   := cur_data_start_idx
  tracker.io.alloc.bits.txn_start_idx    := total_bytes_requested
  tracker.io.alloc.bits.txn_bytes        := cur_txn_bytes
  tracker.io.alloc.bits.txn_log2_bytes   := cur_txn_log2_bytes
  tracker.io.alloc.bits.paddr            := cur_paddr

  // output transaction towards tile-link A-channel
  io.txn.valid       := false.B
  io.txn.bits.xactid := io.nextid

  //-----------------------------------------------
  // FSM
  //-----------------------------------------------
  val (s_IDLE :: 
       s_START_TRANSLATE :: 
       s_FINISH_TRANSLATE :: 
       s_REQ_NEXT_CHUNK :: 
       Nil) = Enum(4)
  val state = RegInit(s_IDLE)

  def init_transfer(dummy : Int = 0) = {
    // TODO: remove this 1-row restriction, (a quite complicated task)
    assert(io.req.bits.lrange.item_rows === 1.U, 
      "cannot request more than 1 row at a time")
    val tmp_vpn = io.req.bits.vaddr(coreMaxAddrBits-1, pgIdxBits)
    val tmp_vpn_mapped  = cur_ppn_valid && (cur_vpn === tmp_vpn)
    val tmp_total_bytes = lrange.total_bytes
    req                := io.req.bits
    total_useful_bytes := tmp_total_bytes
    useful_bytes_left  := tmp_total_bytes
    cur_vaddr          := io.req.bits.vaddr
    state              := Mux(tmp_vpn_mapped, 
                              s_REQ_NEXT_CHUNK, s_START_TRANSLATE)
  }

  switch (state) {
    is (s_IDLE) {
      io.req.ready := true.B
      when (io.req.fire()) {
        init_transfer()
      }
    }
    is (s_START_TRANSLATE) {
      io.tlb.req.valid := true.B
      when (io.tlb.req.fire()) {
        state := s_FINISH_TRANSLATE
      }
    }
    is (s_FINISH_TRANSLATE) {
      io.tlb.resp.ready := true.B
      when (io.tlb.resp.fire()) {
        cur_ppn := io.tlb.resp.bits.paddr(paddrBits-1, pgIdxBits)
        cur_ppn_valid := true.B
        state := s_REQ_NEXT_CHUNK
      }
    }
    is (s_REQ_NEXT_CHUNK) {
      tracker.io.alloc.valid := io.txn.ready
      io.txn.valid           := tracker.io.alloc.ready
      when(io.txn.fire()) {
        val next_vaddr      = cur_vaddr + cur_useful_bytes
        val next_vpn        = next_vaddr(coreMaxAddrBits-1, pgIdxBits)
        val needs_translate = (next_vpn =/= cur_vpn)

        when (is_first_txn) {
          tracker.io.alloc.bits.start_rshift := cur_rshift
          start_rshift                       := cur_rshift
        }
        useful_bytes_left := useful_bytes_left - cur_useful_bytes
        cur_vaddr := next_vaddr

        when (tracker.io.alloc.bits.is_last_txn) {
          state := s_IDLE
          io.req.ready := true.B
          when (io.req.fire()) {
            init_transfer()
          }
        } .elsewhen (needs_translate) {
          state := s_START_TRANSLATE
        }
      }
    }
  }
  //-----------------------------------------------
  // busy signal
  //-----------------------------------------------
  io.busy := tracker.io.busy || (state =/= s_IDLE)
}