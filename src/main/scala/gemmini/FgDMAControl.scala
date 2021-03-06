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
class FgDMAControlRequest[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange(config)
  val status = new MStatus
}

//===========================================================================
// request from FgDMAControl to the circuit that sends tilelink-A reqs
//===========================================================================
class FgDMADispatch[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val xactid         = UInt(DMA_REQS_IDX.W)
  val paddr          = UInt(coreMaxAddrBits.W)
  val txn_log2_bytes = UInt(DMA_TXN_BYTES_CTR_IDX.W)
  val is_last_txn    = Bool()
}

//===========================================================================
// FgDMAControl is same controller for DMA reads/writes
// - it handles translating vaddrs to paddrs
// - it handles splitting operations into tilelink transactions
// - it allocates entries in the FgDMATracker for each tilelink txn initiated
// - achieves max throughput of 1 dispatch/cycle
//===========================================================================
class FgDMAControl[T <: Data]
  (config: FgGemminiArrayConfig[T], max_xfer_bytes: Int, is_load_mode:Boolean)
  (implicit p: Parameters) extends CoreModule with MemoryOpConstants {
  import config._
  //-----------------------------------------------
  // I/O interface
  //-----------------------------------------------
  val io = IO(new Bundle {
    val req        = Flipped(Decoupled(new FgDMAControlRequest(config)))
    val txn_nextid = Input(UInt(DMA_REQS_IDX.W))
    val alloc      = Decoupled(new FgDMATrackerEntry(config, max_xfer_bytes))
    val req_curid  = Input(UInt(DMA_REQS_IDX.W))
    val incr       = Valid(new FgDMAReqIncr(config))
    val dispatch   = Decoupled(new FgDMADispatch(config))
    val tlb        = new FrontendTLBIO
    val flush      = Input(Bool())
  })
  io.req.ready := false.B

  //-----------------------------------------------
  // active request
  //-----------------------------------------------
  val req = Reg(new FgDMAControlRequest(config))
  val mstatus   = req.status
  val lrange    = req.lrange
  val item_rows = lrange.rows
  val item_cols = lrange.cols
  val is_acc    = lrange.is_acc

  // initialized on io.req.fire()
  val total_useful_bytes     = RegInit(0.U(log2Ceil(max_xfer_bytes+1).W))
  val useful_bytes_left      = RegInit(0.U(log2Ceil(max_xfer_bytes+1).W))
  val total_bytes_requested  = RegInit(0.U(log2Ceil(max_xfer_bytes+1).W))
  val useful_bytes_requested = total_useful_bytes - useful_bytes_left

  //-----------------------------------------------
  // TLB Address translation
  //-----------------------------------------------
  val cur_vaddr     = RegInit(0.U(coreMaxAddrBits.W))
  val cur_vpn       = cur_vaddr(coreMaxAddrBits-1, pgIdxBits)
  val cur_ppn       = RegInit(0.U((coreMaxAddrBits - pgIdxBits).W))
  val cur_ppn_valid = withReset(io.flush.asBool()) { RegInit(false.B) }
  val cur_paddr     = Cat(cur_ppn, cur_vaddr(pgIdxBits-1, 0))

  io.tlb.req.valid                    := false.B
  io.tlb.req.bits.tlb_req.vaddr       := Cat(cur_vpn, 0.U(pgIdxBits.W))
  io.tlb.req.bits.tlb_req.passthrough := false.B
  io.tlb.req.bits.tlb_req.size        := 0.U
  io.tlb.req.bits.tlb_req.cmd         := (if(is_load_mode) M_XRD else M_XWR)
  io.tlb.req.bits.status              := mstatus

  //-----------------------------------------------
  // (combinational) Select the size and mask of the TileLink request
  //-----------------------------------------------
  class Txn extends Bundle {
    val bytes          = UInt(DMA_TXN_BYTES_CTR.W)
    val log2_bytes     = UInt(DMA_TXN_BYTES_CTR_IDX.W)
    val useful_bytes   = UInt(DMA_TXN_BYTES_CTR.W)
    val data_start_idx = UInt(DMA_TXN_BYTES_IDX.W)
    val paddr          = UInt(paddrBits.W)
  }

  val candidate_txns = (DMA_BUS_BYTES to DMA_TXN_BYTES by DMA_BUS_BYTES)
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
    Mux(cur.useful_bytes === best.useful_bytes, 
      Mux(cur.log2_bytes < best.log2_bytes, cur, best),
      Mux(cur.useful_bytes > best.useful_bytes, cur, best))
  }
  val cur_txn_useful_bytes = best_txn.useful_bytes
  val cur_txn_bytes        = best_txn.bytes
  val cur_txn_log2_bytes   = best_txn.log2_bytes
  val cur_data_start_idx   = best_txn.data_start_idx
  val cur_aligned_paddr    = best_txn.paddr

  //-----------------------------------------------
  // allocate new tag for the tile-link request
  //-----------------------------------------------
  val first_txn_data_start_idx = RegInit(0.U(DMA_TXN_BYTES_IDX.W))
  val is_last_txn              = (cur_txn_useful_bytes === useful_bytes_left)
  val is_first_txn             = (useful_bytes_requested === 0.U)

  io.alloc.valid                 := false.B
  io.alloc.bits.reqid            := io.req_curid
  io.alloc.bits.lrange           := lrange
  io.alloc.bits.req_useful_bytes := total_useful_bytes
  io.alloc.bits.data_start_idx   := first_txn_data_start_idx
  io.alloc.bits.txn_start_idx    := total_bytes_requested
  io.alloc.bits.txn_useful_bytes := cur_txn_useful_bytes   
  io.alloc.bits.txn_bytes        := cur_txn_bytes
  io.alloc.bits.is_last_txn      := is_last_txn

  // output transaction towards tile-link A-channel
  io.dispatch.valid               := false.B
  io.dispatch.bits.xactid         := io.txn_nextid
  io.dispatch.bits.paddr          := cur_aligned_paddr
  io.dispatch.bits.txn_log2_bytes := cur_txn_log2_bytes

  // increment transactions pending for this request
  io.incr.valid              := false.B
  io.incr.bits.reqid         := io.req_curid
  io.incr.bits.all_txns_sent := is_last_txn

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
    assert(io.req.bits.lrange.rows === 1.U, 
      "cannot request more than 1 row at a time")
    val tmp_total_bytes = if(is_load_mode)
                             io.req.bits.lrange.total_write_bytes() else
                             io.req.bits.lrange.total_read_bytes()
    val next_vaddr      = io.req.bits.vaddr
    val next_vpn        = next_vaddr(coreMaxAddrBits-1, pgIdxBits)
    val needs_translate = (next_vpn =/= cur_vpn) || !cur_ppn_valid

    req                   := io.req.bits
    total_bytes_requested := 0.U
    total_useful_bytes    := tmp_total_bytes
    useful_bytes_left     := tmp_total_bytes
    cur_vaddr             := next_vaddr
    cur_ppn_valid         := !needs_translate
    state                 := s_REQ_NEXT_CHUNK

    // initiate TLB request if possible
    when (needs_translate) {
      state := s_START_TRANSLATE
      io.tlb.req.valid := true.B
      io.tlb.req.bits.tlb_req.vaddr := Cat(next_vpn, 0.U(pgIdxBits.W))
      when (io.tlb.req.fire()) {
        state := s_FINISH_TRANSLATE
      }
    }
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
      when (io.tlb.resp.fire()) {
        cur_ppn := io.tlb.resp.bits.paddr(paddrBits-1, pgIdxBits)
        cur_ppn_valid := true.B
        state := s_REQ_NEXT_CHUNK
      }
    }
    is (s_REQ_NEXT_CHUNK) {
      io.dispatch.valid := io.alloc.ready
      io.alloc.valid    := io.dispatch.ready
      when(io.dispatch.fire()) {
        io.incr.valid := true.B

        val next_vaddr      = cur_vaddr + cur_txn_useful_bytes
        val next_vpn        = next_vaddr(coreMaxAddrBits-1, pgIdxBits)
        val needs_translate = (next_vpn =/= cur_vpn)

        when (is_first_txn) {
          io.alloc.bits.data_start_idx := cur_data_start_idx
          first_txn_data_start_idx     := cur_data_start_idx
        }
        
        total_bytes_requested := total_bytes_requested + cur_txn_bytes
        useful_bytes_left     := useful_bytes_left - cur_txn_useful_bytes
        cur_vaddr             := next_vaddr
        cur_ppn_valid         := !needs_translate

        when (is_last_txn) {
          state := s_IDLE
          io.req.ready := true.B
          when (io.req.fire()) {
            init_transfer()
          }
        } .elsewhen (needs_translate) {
          state := s_START_TRANSLATE
          io.tlb.req.valid := true.B
          io.tlb.req.bits.tlb_req.vaddr := Cat(next_vpn, 0.U(pgIdxBits.W))
          when (io.tlb.req.fire()) {
            state := s_FINISH_TRANSLATE
          }
        }
      }
    }
  }
}
