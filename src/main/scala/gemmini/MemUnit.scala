package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink.{TLIdentityNode, TLXbar}
import Util._

//===========================================================================
// MemOpController <-> MemUnit Interfaces (decoupled)
//===========================================================================
class MemUnitMemOpReq[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new LocalRange(config)
  val status = new MStatus
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

class MemUnitMemOpResp[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

class MemUnitMemOpIO[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val req  = Decoupled(new MemUnitMemOpReq(config))
  val resp = Flipped(Decoupled(new MemUnitMemOpResp(config)))
}

//===========================================================================
// ExecController <-> MemUnit (non-decoupled)
//===========================================================================
class MemUnitExecReadReq[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val en   = Output(Bool())
  val row  = Output(UInt(SP_ROWS_IDX.W))
  val cols = Output(UInt(DIM_CTR.W))
}

class MemUnitExecReadResp[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val data = Output(UInt(SP_ROW_BITS.W))
}

class MemUnitExecReadIO[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val req  = new MemUnitExecReadReq(config)
  val resp = Flipped(new MemUnitExecReadResp(config))
}

class MemUnitExecWriteReq[T <: Data](val config: GemminiArrayConfig[T]) 
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en    = Output(Bool())
  val row   = Output(UInt(SP_ROWS_IDX.W))
  val cols  = Output(UInt(DIM_CTR.W))
  val accum = Output(Bool())
  val data  = Output(UInt(ACC_ROW_BITS.W))
}

//============================================================================
// MemUnit - interfaces DMA, ExecUnit to the local scratchpad/accumulators
//============================================================================
class MemUnit[T <: Data: Arithmetic](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends LazyModule with HasCoreParameters {
  import config._

  val id_node = TLIdentityNode()
  val xbar_node = TLXbar()

  val load  = LazyModule(new DMALoad(config))
  val store = LazyModule(new DMAStore(config))

  xbar_node := load.node
  xbar_node := store.node
  id_node   := xbar_node

  override lazy val module = new MemUnitModuleImp(this)
}

class MemUnitModuleImp[T <: Data: Arithmetic](outer: MemUnit[T])
  (implicit p: Parameters)
  extends LazyModuleImp(outer) with HasCoreParameters {
  import outer.{config,load,store}
  import config._
  //------------------------------------------
  // I/O interface
  //------------------------------------------
  val io = IO(new Bundle {
    val dma = new Bundle {
      val load  = Flipped(new MemUnitMemOpIO(config))
      val store = Flipped(new MemUnitMemOpIO(config))
    }
    val tlb = new Bundle {
      val load  = new FrontendTLBIO
      val store = new FrontendTLBIO
    }
    val exec = new Bundle {
      val read  = Vec(2, Flipped(new MemUnitExecReadIO(config)))
      val write = Flipped(new MemUnitExecWriteReq(config))
    }
    val acc_config = Flipped(new AccumulatorBankConfigIO(config))
    val busy = Output(Bool())
    val flush = Input(Bool())
  })

  //--------------------------------------------
  // setup tile-link DMA load/stores
  //--------------------------------------------
  load.module.io.chunk.ready := false.B
  load.module.io.req         <> io.dma.load.req
  io.dma.load.resp           <> load.module.io.resp
       
  io.tlb.load  <> load.module.io.tlb
  io.tlb.store <> store.module.io.tlb

  load.module.io.flush  := io.flush
  store.module.io.flush := io.flush

  io.busy := load.module.io.busy || store.module.io.busy

  //========================================================================
  {
    val banks = Seq.fill(SP_BANKS) { Module(new ScratchpadBank(config)) }
    banks.foreach { bank => 
      bank.io.read.req.en := false.B 
      bank.io.write.en    := false.B 
    }

    //--------------------------------------
    // read-datapath (2-cycle latency non-decoupled)
    // - NOTE: exec-unit is responsible for preventing bank conflicts!
    //--------------------------------------
    io.exec.read.foreach { rio => 
      val ex_rd_req  = rio.req
      val ex_rd_en   = ex_rd_req.en
      val ex_rd_bank = ex_rd_req.row(SP_ROWS_IDX-1,SP_BANK_ROWS_IDX)
      val ex_rd_row  = ex_rd_req.row(SP_BANK_ROWS_IDX-1,0)
      val ex_rd_mask = ((1.U<<ex_rd_req.cols)-1.U).asTypeOf(Vec(DIM, Bool()))

      // datapath into scratchpads
      banks(ex_rd_bank).io.read.req.en  := ex_rd_en
      banks(ex_rd_bank).io.read.req.row := ex_rd_row

      // forward configs 1-cycle to match scratchpad latency
      val ex_rd_en_buf   = ShiftRegister(ex_rd_en,   1)
      val ex_rd_bank_buf = ShiftRegister(ex_rd_bank, 1)
      val ex_rd_mask_buf = ShiftRegister(ex_rd_mask, 1)

      val ex_rd_data_pre = banks(ex_rd_bank_buf).io.read.resp.data.asTypeOf(
                            Vec(DIM, inputType))
      val ex_rd_data = WireDefault(0.U.asTypeOf(Vec(DIM, inputType)))

      for (i <- 0 until DIM) {
        when(ex_rd_en_buf && ex_rd_mask_buf(i)) {
          ex_rd_data(i) := ex_rd_data_pre(i)
        }
      }
      // return the zero-padded data 1 cycle later
      rio.resp.data := RegNext(ex_rd_data)
    }

    //--------------------------------------
    // write-datapath
    //--------------------------------------
    val dma_wr_fire   = load.module.io.chunk.fire()
    val dma_wr_data   = load.module.io.chunk.bits.data
    val dma_wr_mask   = load.module.io.chunk.bits.mask
    val dma_wr_lrange = load.module.io.chunk.bits.lrange
    val dma_wr_row    = dma_wr_lrange.row_start_within_bank()
    val dma_wr_bank   = dma_wr_lrange.bank_start()

    when (!dma_wr_lrange.is_acc) {
      load.module.io.chunk.ready := true.B

      when (dma_wr_fire) {
        assert(!dma_rd_lrange.is_acc)
        assert(!dma_rd_lrange.is_accum)
        assert(!dma_rd_lrange.is_garbage)
        assert(dma_rd_lrange.rows === 1)
        assert(dma_rd_lrange.row_start < SP_ROWS)

        // datapath into scratchpads
        banks(dma_wr_bank).io.write.en   := true.B
        banks(dma_wr_bank).io.write.row  := dma_wr_row
        banks(dma_wr_bank).io.write.mask := dma_wr_mask
        banks(dma_wr_bank).io.write.data := dma_wr_data(SP_ROW_BITS-1,0)
      }
    }
  }

  //========================================================================
  {
    val banks = Seq.fill(ACC_BANKS) { Module(new AccumulatorBank(config)) }
    banks.foreach { bank => 
      bank.io.acc_config  := io.acc_config 
      bank.io.read.req.en := false.B 
      bank.io.write.en    := false.B 
    }
 
    //--------------------------------------
    // write-datapath
    //--------------------------------------
    val ex_wr_req   = io.exec.write
    val ex_wr_en    = ex_wr_req.en
    val ex_wr_bank  = ex_wr_req.row(ACC_ROWS_IDX-1,ACC_BANK_ROWS_IDX)
    val ex_wr_row   = ex_wr_req.row(ACC_BANK_ROWS_IDX-1,0)
    val ex_wr_accum = ex_wr_req.accum
    val ex_wr_mask  = ((1.U<<ex_wr_req.cols)-1.U).asTypeOf(Vec(DIM, Bool()))
    val ex_wr_data  = ex_wr_req.data

    // datapath into scratchpads
    when (ex_wr_en) {
      banks(ex_wr_bank).io.write.en    := true.B
      banks(ex_wr_bank).io.write.row   := ex_wr_row
      banks(ex_wr_bank).io.write.accum := ex_wr_accum
      banks(ex_wr_bank).io.write.mask  := ex_wr_mask
      banks(ex_wr_bank).io.write.data  := ex_wr_data
    }

    val dma_wr_fire   = load.module.io.chunk.fire()
    val dma_wr_data   = load.module.io.chunk.bits.data
    val dma_wr_mask   = load.module.io.chunk.bits.mask
    val dma_wr_lrange = load.module.io.chunk.bits.lrange
    val dma_wr_row    = dma_wr_lrange.row_start_within_bank()
    val dma_wr_bank   = dma_wr_lrange.bank_start()

    when (dma_wr_lrange.is_acc) {
      // execute write takes precedence over dma load to same bank
      load.module.io.chunk.ready := !ex_wr_req || (dma_wr_bank != ex_wr_bank)
    }
    when (dma_wr_fire) {
      assert(dma_wr_lrange.is_acc)
      assert(!dma_wr_lrange.is_accum) // dma load doesn't accumulate
      assert(!dma_wr_lrange.is_garbage)
      assert(dma_wr_lrange.rows === 1)
      assert(dma_wr_bank < ACC_BANKS)

      // datapath into scratchpads
      banks(dma_wr_bank).io.write.en    := true.B
      banks(dma_wr_bank).io.write.row   := dma_wr_row
      banks(dma_wr_bank).io.write.accum := false.B
      banks(dma_wr_bank).io.write.mask  := dma_wr_mask
      banks(dma_wr_bank).io.write.data  := dma_wr_data
    }

    //--------------------------------------
    // read-datapath (store)
    // - can't read when a write is issued.
    //--------------------------------------
    val dma_rd_req       = io.dma.store.req
    val dma_rd_vaddr     = dma_rd_req.bits.vaddr
    val dma_rd_status    = dma_rd_req.bits.status
    val dma_rd_rob_id    = dma_rd_req.bits.rob_id
    val dma_rd_lrange    = dma_rd_req.bits.lrange
    val dma_rd_rows      = dma_rd_lrange.rows
    val dma_rd_row       = dma_rd_lrange.row_start_within_bank()
    val dma_rd_bank      = dma_rd_lrange.bank_start()

    val store_is_blocked = WireDefault(false.B)
    io.dma.store.req.ready := !banks(dma_rd_bank).io.write.en &&
                              !store_is_blocked

    val dma_rd_fire = io.dma.store.req.fire()
    when (dma_rd_fire) {
      assert(dma_rd_lrange.is_acc)
      assert(!dma_rd_lrange.is_accum)
      assert(!dma_rd_lrange.is_garbage)
      assert(!dma_rd_lrange.is_im2col) // no store im2col support yet
      assert(dma_rd_rows === 1.U)
      assert(dma_rd_bank < ACC_BANKS)

      // bank datapath
      banks(ex_rd_bank).io.read.req.en  := true.B
      banks(ex_rd_bank).io.read.req.row := dma_rd_row
    }

    // match 2-cycle latency of accumulator read
    val dma_rd_en_buf     = ShiftRegister(dma_rd_fire,   2)
    val dma_rd_bank_buf   = ShiftRegister(dma_rd_bank,   2)
    val dma_rd_vaddr_buf  = ShiftRegister(dma_rd_vaddr,  2)
    val dma_rd_lrange_buf = ShiftRegister(dma_rd_lrange, 2)
    val dma_rd_status_buf = ShiftRegister(dma_rd_status, 2)
    val dma_rd_rob_id_buf = ShiftRegister(dma_rd_rob_id, 2)

    val dma_rd_data = banks(dma_rd_bank_buf).io.read.resp.data
    
    //---------------------------------------------
    // bankend of store datapath: handoff data to store-dma engine
    //---------------------------------------------
    val dma_rd_q = Module(new Queue(new DMAStoreRequest(config), 4))
    dma_rd_q.io.deq.ready := false.B

    // we can only be SURE that the io.dma.store.req can be handled 2 cycles
    // later if the queue has a slot open for it if the dma engine blocks
    store_is_blocked := (dma_rd_q.io.count > 1.U)

    // queue accumulator read if can't send to dma-store unit this cycle
    dma_rd_q.io.enq.valid := dma_rd_en_buf && (!store.module.io.req.ready || 
                                               dma_rd_q.io.deq.valid)
    dma_rd_q.io.enq.bits.data   := dma_rd_data
    dma_rd_q.io.enq.bits.vaddr  := dma_rd_vaddr_buf
    dma_rd_q.io.enq.bits.lrange := dma_rd_lrange_buf
    dma_rd_q.io.enq.bits.status := dma_rd_status_buf
    dma_rd_q.io.enq.bits.rob_id := dma_rd_rob_id_buf

    // send the accumulator read data or queued data to dma-store if ready
    when (dma_rd_q.io.deq.valid) {
      store.module.io.req <> dma_rd_q.io.deq
    } .otherwise {
      store.module.io.req.valid       := dma_rd_en_buf
      store.module.io.req.bits.data   := dma_rd_data
      store.module.io.req.bits.vaddr  := dma_rd_vaddr_buf
      store.module.io.req.bits.lrange := dma_rd_lrange_buf
      store.module.io.req.bits.status := dma_rd_status_buf
      store.module.io.req.bits.rob_id := dma_rd_rob_id_buf
    }

    // internal dma-store decoupled response to external store-controller
    store.module.io.resp.ready    := io.dma.store.resp.ready
    io.dma.store.resp.valid       := store.module.io.resp.valid
    io.dma.store.resp.bits.rob_id := store.module.io.resp.bits.rob_id
  }
}
