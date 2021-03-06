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
// MemOpController <-> MemUnit Interfaces
//===========================================================================
class FgMemUnitMemOpReq[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange(config)
  val status = new MStatus
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

class FgMemUnitMemOpResp[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

class FgMemUnitMemOpIO[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val req  = Decoupled(new FgMemUnitMemOpReq(config))
  val resp = Flipped(Decoupled(new FgMemUnitMemOpResp(config)))
}

//===========================================================================
// ExecController <-> MemUnit
//===========================================================================
class FgMemUnitExecReadReq[T <: Data]
  (val config: FgGemminiArrayConfig[T], val fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en         = Output(Bool())
  val row        = Output(UInt(FG_DIM_IDX.W))
  val col_start  = Output(UInt(log2Up(fg_cols * FG_DIM).W))
  val bank_start = Output(UInt(FG_NUM_IDX.W))
  val banks      = Output(UInt(FG_NUM_CTR.W))
}

class FgMemUnitExecReadResp[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val data = Output(UInt(AB_EXEC_PORT_BITS.W))
}

class FgMemUnitExecReadIO[T <: Data]
  (val config: FgGemminiArrayConfig[T], val fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  val req  = new FgMemUnitExecReadReq(config, fg_cols)
  val resp = Flipped(new FgMemUnitExecReadResp(config))
}

class FgMemUnitExecWriteReq[T <: Data](val config: FgGemminiArrayConfig[T]) 
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en         = Output(Bool())
  val row        = Output(UInt(FG_DIM_IDX.W))
  val cols       = Output(UInt(CD_ACC_ROW_ELEMS_CTR.W))
  val col_start  = Output(UInt(CD_ACC_ROW_ELEMS_IDX.W))
  val bank_start = Output(UInt(FG_NUM_IDX.W))
  val banks      = Output(UInt(FG_NUM_CTR.W))
  val accum      = Output(Bool())
  val data       = Output(UInt(D_LOAD_ROW_BITS.W))
}

//============================================================================
// FgMemUnit
// - supports multiple concurrent fine-grained systolic-array reads/writes
//============================================================================
class FgMemUnit[T <: Data: Arithmetic](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends LazyModule with HasCoreParameters {
  import config._

  val id_node = TLIdentityNode()
  val xbar_node = TLXbar()

  val loadA  = LazyModule(new FgDMALoad(config, "loadA", A_SP_ROW_BYTES))
  val loadB  = LazyModule(new FgDMALoad(config, "loadB", B_SP_ROW_BYTES))
  val loadD  = LazyModule(new FgDMALoad(config, "loadD", D_LOAD_ROW_BYTES))
  val storeC = LazyModule(new FgDMAStore(config,"storeC",C_STORE_ROW_BYTES))

  xbar_node := loadA.node
  xbar_node := loadB.node
  xbar_node := loadD.node
  xbar_node := storeC.node
  id_node   := xbar_node

  override lazy val module = new FgMemUnitModuleImp(this)
}

class FgMemUnitModuleImp[T <: Data: Arithmetic](outer: FgMemUnit[T])
  (implicit p: Parameters)
  extends LazyModuleImp(outer) with HasCoreParameters {
  import outer.{config,loadA,loadB,loadD,storeC}
  import config._
  //------------------------------------------
  // I/O interface
  //------------------------------------------
  val io = IO(new Bundle {
    val dma = new Bundle {
      val loadA  = Flipped(new FgMemUnitMemOpIO(config))
      val loadB  = Flipped(new FgMemUnitMemOpIO(config))
      val loadD  = Flipped(new FgMemUnitMemOpIO(config))
      val storeC = Flipped(new FgMemUnitMemOpIO(config))
    }
    val tlb = new Bundle {
      val loadA  = new FrontendTLBIO
      val loadB  = new FrontendTLBIO
      val loadD  = new FrontendTLBIO
      val storeC = new FrontendTLBIO
    }
    val exec = new Bundle {
      val readA  = Flipped(new FgMemUnitExecReadIO(config, A_SP_FG_COLS))
      val readB  = Flipped(new FgMemUnitExecReadIO(config, B_SP_FG_COLS))
      val writeC = Flipped(new FgMemUnitExecWriteReq(config))
    }
    val acc_config = Flipped(new FgAccumulatorBankConfigIO(config))
    val busy = Output(Bool())
    val flush = Input(Bool())
    // set by the TilerController
    val a_fg_mux_sel = Input(UInt(FG_NUM_CTR_CTR.W))
  })

  //--------------------------------------------
  // setup tile-link DMA load/stores
  //--------------------------------------------
  Seq(loadA.module.io, loadB.module.io, loadD.module.io).zip(
    Seq(io.dma.loadA, io.dma.loadB, io.dma.loadD)).foreach { case(dma,io) => 
      dma.req <> io.req
      io.resp <> dma.resp
    }
       
  io.tlb.loadA  <> loadA.module.io.tlb
  io.tlb.loadB  <> loadB.module.io.tlb
  io.tlb.loadD  <> loadD.module.io.tlb
  io.tlb.storeC <> storeC.module.io.tlb

  loadA.module.io.flush  := io.flush
  loadB.module.io.flush  := io.flush
  loadD.module.io.flush  := io.flush
  storeC.module.io.flush := io.flush

  io.busy := loadA.module.io.busy || loadB.module.io.busy || 
             loadD.module.io.busy || storeC.module.io.busy

  //========================================================================
  {
    // A-banks
    val banks = Seq.fill(A_SP_BANKS) { 
      Module(new FgScratchpadBank(config, A_SP_FG_COLS, A_SP_PORT_FG_COLS)) 
    }

    //--------------------------------------
    // read-datapath (2-cycle non-decoupled)
    // - exec reads can read from 1+ banks at a time
    //--------------------------------------
    val ex_rd_req        = io.exec.readA.req
    val ex_rd_en         = ex_rd_req.en
    val ex_rd_row        = ex_rd_req.row
    val ex_rd_col_start  = ex_rd_req.col_start
    val ex_rd_bank_start = ex_rd_req.bank_start
    val ex_rd_banks      = ex_rd_req.banks
    when (ex_rd_en) {
      assert((ex_rd_bank_start % ex_rd_banks) === 0.U)
    }

    // forward configs 2-cycle to match scratchpad latency
    val ex_rd_bank_start_buf = ShiftRegister(ex_rd_bank_start, 2)

    // datapath into scratchpads
    banks.zipWithIndex.foreach { case (bank, i) =>
      bank.io.read.req.en        := ex_rd_en
      bank.io.read.req.row       := ex_rd_row
      bank.io.read.req.col_start := ex_rd_col_start
    }

    // read datapath out of scratchpads and output data (2 cycle latency)
    val ex_rd_datas     = Cat(banks.reverse.map { 
                              bank => bank.io.read.resp.data })
    val ex_rd_data_cast = ex_rd_datas.asTypeOf(UInt(AB_EXEC_PORT_BITS.W))
    val ex_rd_bitshift  = ex_rd_bank_start_buf * FG_DIM.U * ITYPE_BITS.U
    io.exec.readA.resp.data := (ex_rd_data_cast >> ex_rd_bitshift)

    //--------------------------------------
    // write-datapath
    //--------------------------------------
    loadA.module.io.chunk.ready := true.B

    val dma_wr_fire      = loadA.module.io.chunk.fire()
    val dma_wr_data      = loadA.module.io.chunk.bits.data
    val dma_wr_lrange    = loadA.module.io.chunk.bits.lrange
    val dma_wr_rows      = dma_wr_lrange.rows
    val dma_wr_row       = dma_wr_lrange.row_start_within_bank()
    val dma_wr_bank      = dma_wr_lrange.bank_start()
    val dma_wr_cols      = dma_wr_lrange.cols
    val dma_wr_col_start = dma_wr_lrange.col_start
    
    when (dma_wr_fire) {
      assert(dma_wr_rows === 1.U, "dma cannot write >1 row per request")
      assert(dma_wr_bank < FG_NUM.U)
    }

    banks.zipWithIndex.foreach { case (bank, i) =>
      bank.io.write.en        := (dma_wr_fire && (dma_wr_bank === i.U))
      bank.io.write.row       := dma_wr_row
      bank.io.write.cols      := dma_wr_cols
      bank.io.write.col_start := dma_wr_col_start
      bank.io.write.data      := dma_wr_data
    }
  }

  //========================================================================
  {
    // B-banks
    val banks = Seq.fill(B_SP_BANKS) { 
      Module(new FgScratchpadBank(config, B_SP_FG_COLS, B_SP_PORT_FG_COLS))
    }

    //--------------------------------------
    // read-datapath (2-cycles non-decoupled)
    //--------------------------------------
    // exec reads can read from 1+ banks at a time
    val ex_rd_req        = io.exec.readB.req
    val ex_rd_en         = ex_rd_req.en
    val ex_rd_row        = ex_rd_req.row
    val ex_rd_col_start  = ex_rd_req.col_start
    val ex_rd_bank_start = ex_rd_req.bank_start
    val ex_rd_banks      = ex_rd_req.banks
    when (ex_rd_en) {
      assert(ex_rd_banks === 1.U && ex_rd_bank_start < 2.U)
    }

    // forward configs 2-cycle to match scratchpad latency
    val ex_rd_bank_start_buf = ShiftRegister(ex_rd_bank_start, 2)

    // datapath into scratchpads
    banks.zipWithIndex.foreach { case (bank, i) =>
      bank.io.read.req.en        := ex_rd_en
      bank.io.read.req.row       := ex_rd_row
      bank.io.read.req.col_start := ex_rd_col_start
    }

    // read datapath out of scratchpads and output data (2 cycle latency)
    val ex_rd_datas = banks.map { bank => bank.io.read.resp.data }
    io.exec.readB.resp.data := Mux1H(ex_rd_datas.zipWithIndex.map { 
      case (data, i) => ((ex_rd_bank_start_buf === i.U) -> data)
    })

    //--------------------------------------
    // write-datapath (decoupled)
    //--------------------------------------
    loadB.module.io.chunk.ready := true.B

    val dma_wr_fire      = loadB.module.io.chunk.fire()
    val dma_wr_data      = loadB.module.io.chunk.bits.data
    val dma_wr_lrange    = loadB.module.io.chunk.bits.lrange
    val dma_wr_rows      = dma_wr_lrange.rows
    val dma_wr_row       = dma_wr_lrange.row_start_within_bank()
    val dma_wr_bank      = dma_wr_lrange.bank_start()
    val dma_wr_cols      = dma_wr_lrange.cols
    val dma_wr_col_start = dma_wr_lrange.col_start

    when (dma_wr_fire) {
      assert(dma_wr_rows === 1.U, "dma cannot write >1 row per request")
      assert(dma_wr_bank < 2.U)
    }

    banks.zipWithIndex.foreach { case (bank, i) =>
      bank.io.write.en        := (dma_wr_fire && (dma_wr_bank === i.U))
      bank.io.write.row       := dma_wr_row
      bank.io.write.cols      := dma_wr_cols
      bank.io.write.col_start := dma_wr_col_start
      bank.io.write.data      := dma_wr_data
    }
  }

  //========================================================================
  {
    // C/D-banks
    val banks = Seq.fill(FG_NUM) { Module(new FgAccumulatorBank(config)) }
    banks.foreach { bank => bank.io.acc_config := io.acc_config }
 
    //--------------------------------------
    // write-datapath
    //--------------------------------------
    val ex_wr_req        = io.exec.writeC
    val ex_wr_en         = ex_wr_req.en
    val ex_wr_row        = ex_wr_req.row
    val ex_wr_cols       = ex_wr_req.cols
    val ex_wr_col_start  = ex_wr_req.col_start
    val ex_wr_banks      = ex_wr_req.banks
    val ex_wr_bank_start = ex_wr_req.bank_start
    val ex_wr_bank_end   = ex_wr_bank_start + ex_wr_banks - 1.U
    val ex_wr_accum      = ex_wr_req.accum
    val ex_wr_data       = ex_wr_req.data
    when (ex_wr_en) {
      assert((ex_wr_bank_start % ex_wr_banks) === 0.U, "banks not aligned")
    }

    // dma-loadD
    val dma_wr_fire      = loadD.module.io.chunk.fire()
    val dma_wr_data      = loadD.module.io.chunk.bits.data
    val dma_wr_lrange    = loadD.module.io.chunk.bits.lrange
    val dma_wr_rows      = dma_wr_lrange.rows
    val dma_wr_row       = dma_wr_lrange.row_start_within_bank()
    val dma_wr_bank      = dma_wr_lrange.bank_start()
    val dma_wr_cols      = dma_wr_lrange.cols
    val dma_wr_col_start = dma_wr_lrange.col_start
    val dma_wr_accum     = false.B

    loadD.module.io.chunk.ready := !ex_wr_en ||
                                   (ex_wr_bank_start > dma_wr_bank) ||
                                   (ex_wr_bank_end < dma_wr_bank)

    when (dma_wr_fire) {
      assert(dma_wr_rows === 1.U, "dma cannot write >1 row at a time")
    }

    val is_writing = ex_wr_en || dma_wr_fire
    banks.zipWithIndex.foreach { case (bank, i) =>
      when (ex_wr_en && (ex_wr_bank_start<=i.U) && (i.U<=ex_wr_bank_end)) {
        val is_active      = true.B
        val bank_offset    = i.U - ex_wr_bank_start
        val max_bank_count = 1.U << io.a_fg_mux_sel
        val bits_per_bank  = (FG_NUM * FG_DIM * OTYPE_BITS).U / max_bank_count
        val shifted_data   = ex_wr_data >> (bank_offset * bits_per_bank)

        bank.io.write.en        := is_active
        bank.io.write.row       := ex_wr_row
        bank.io.write.cols      := ex_wr_cols
        bank.io.write.col_start := ex_wr_col_start
        bank.io.write.data      := shifted_data
        bank.io.write.accum     := ex_wr_accum
      } .otherwise {
        bank.io.write.en        := dma_wr_fire && (dma_wr_bank === i.U)
        bank.io.write.row       := dma_wr_row
        bank.io.write.cols      := dma_wr_cols
        bank.io.write.col_start := dma_wr_col_start
        bank.io.write.data      := dma_wr_data
        bank.io.write.accum     := dma_wr_accum
      }
    }

    //--------------------------------------
    // read-datapath (storeC)
    // - accept read req if we know a spot is reserved for it 2 cycles later
    // - acc bank read behavior:
    //   - col_start must be self-aligned
    //   - total fg_cols per row written === FG_NUM / total_banks
    //   - dma_store only reads 1 row of 1 bank at a time
    //   - dma_load only writes 1 row of 1 bank at a time
    //   - ex_write writes 1 row of multiple consecutive banks at a time
    //   - simulatenously read banks are concatenated
    // - datapath into scratchpads
    //--------------------------------------
    val dma_rd_req       = io.dma.storeC.req
    val dma_rd_vaddr     = dma_rd_req.bits.vaddr
    val dma_rd_status    = dma_rd_req.bits.status
    val dma_rd_rob_id    = dma_rd_req.bits.rob_id
    val dma_rd_lrange    = dma_rd_req.bits.lrange
    val dma_rd_rows      = dma_rd_lrange.rows
    val dma_rd_row       = dma_rd_lrange.row_start_within_bank()
    val dma_rd_bank      = dma_rd_lrange.bank_start()
    val dma_rd_cols      = dma_rd_lrange.cols
    val dma_rd_col_start = dma_rd_lrange.col_start

    val store_is_blocked = WireDefault(false.B)
    io.dma.storeC.req.ready := MuxCase(false.B, 
      banks.zipWithIndex.map { case(bank, i) => 
        (dma_rd_bank === i.U) -> (!bank.io.write.en && !store_is_blocked)
      })
    val dma_rd_fire = io.dma.storeC.req.fire()
    when (dma_rd_fire) {
      assert(dma_rd_rows === 1.U, "dma cannot read >1 acc rows per request")
    }

    banks.zipWithIndex.foreach { case (bank, i) =>
      bank.io.read.req.en        := dma_rd_fire && (dma_rd_bank === i.U)
      bank.io.read.req.row       := dma_rd_row
      bank.io.read.req.col_start := dma_rd_col_start
    }
    val dma_rd_en_buf     = ShiftRegister(dma_rd_fire,   2)
    val dma_rd_cols_buf   = ShiftRegister(dma_rd_cols,   2)
    val dma_rd_bank_buf   = ShiftRegister(dma_rd_bank,   2)
    val dma_rd_vaddr_buf  = ShiftRegister(dma_rd_vaddr,  2)
    val dma_rd_lrange_buf = ShiftRegister(dma_rd_lrange, 2)
    val dma_rd_status_buf = ShiftRegister(dma_rd_status, 2)
    val dma_rd_rob_id_buf = ShiftRegister(dma_rd_rob_id, 2)

    // read datapath out of scratchpads and output data (2 cycle latency)
    val dma_rd_datas = VecInit(banks.map { bank => bank.io.read.resp.data })
    val dma_rd_data  = dma_rd_datas(dma_rd_bank_buf)

    val dma_rd_q_type = new FgDMAStoreRequest(config, C_STORE_ROW_BYTES)
    val dma_rd_q = Module(new Queue(dma_rd_q_type, 4))
    dma_rd_q.io.deq.ready := false.B

    // we can only be SURE that the io.dma.storeC.req can be handled 2 cycles
    // later if the queue has a slot open for it if the dma engine blocks
    store_is_blocked := (dma_rd_q.io.count > 1.U)

    // queue accumulator read if can't send to dma-store unit this cycle
    dma_rd_q.io.enq.valid := dma_rd_en_buf && (!storeC.module.io.req.ready || 
                                               dma_rd_q.io.deq.valid)
    dma_rd_q.io.enq.bits.data   := dma_rd_data
    dma_rd_q.io.enq.bits.vaddr  := dma_rd_vaddr_buf
    dma_rd_q.io.enq.bits.lrange := dma_rd_lrange_buf
    dma_rd_q.io.enq.bits.status := dma_rd_status_buf
    dma_rd_q.io.enq.bits.rob_id := dma_rd_rob_id_buf

    // send the accumulator read data or queued data to dma-store if ready
    when (dma_rd_q.io.deq.valid) {
      storeC.module.io.req <> dma_rd_q.io.deq
    } 
    .otherwise {
      storeC.module.io.req.valid       := dma_rd_en_buf
      storeC.module.io.req.bits.data   := dma_rd_data.asUInt()
      storeC.module.io.req.bits.vaddr  := dma_rd_vaddr_buf
      storeC.module.io.req.bits.lrange := dma_rd_lrange_buf
      storeC.module.io.req.bits.status := dma_rd_status_buf
      storeC.module.io.req.bits.rob_id := dma_rd_rob_id_buf
    }

    // internal dma-store decoupled response to external store-controller
    storeC.module.io.resp.ready    := io.dma.storeC.resp.ready
    io.dma.storeC.resp.valid       := storeC.module.io.resp.valid
    io.dma.storeC.resp.bits.rob_id := storeC.module.io.resp.bits.rob_id
  }
}

