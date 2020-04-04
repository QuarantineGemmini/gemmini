package gemmini

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths}

import chisel3._
import chisel3.util._
import chisel3.experimental._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import GemminiISA._


object DataType extends Enumeration {
  val I32, F64 = Value
}

class CmdTilerInterface extends Bundle {
  // Config Data
  val addr_a = Output(UInt(64.W)) // FIXME: is width a param somewhere?
  val addr_b = Output(UInt(64.W))
  val addr_c = Output(UInt(64.W))
  val addr_d = Output(UInt(64.W))

  val m = Output(UInt(64.W))
  val k = Output(UInt(64.W))
  val n = Output(UInt(64.W))

  val dataflow = Output(Bool()) 
  val activation = Output(UInt(2.W)) 
  
  val matmul_rshift = Output(UInt(32.W))
  val acc_rshift = Output(UInt(32.W))
  val relu6_lshift = Output(UInt(32.W))
  val repeating_bias = Output(Bool())
  
  val dtype = Output(UInt(64.W)) // FIXME: need an instruction for this. Zero for now
}

class GemminiCmdWithDeps(rob_entries: Int)(implicit p: Parameters) extends Bundle {
  val cmd = new RoCCCommand
  val rob_id = UInt(log2Up(rob_entries).W)

  override def cloneType: this.type = (new GemminiCmdWithDeps(rob_entries)).asInstanceOf[this.type]
}

class CmdFsm (implicit p: Parameters) extends Module {
  // Command FSM 
  // * Receive RoCC commands 
  // * Check validity of configuration data 
  // * Hand off to Tiling on COMPUTE requests 

  val io = IO(new Bundle {
    val cmd = Flipped(Decoupled(new RoCCCommand))
    val tiler = Decoupled(new CmdTilerInterface)
  });
  
  object CmdFsmState extends ChiselEnum {
    val LISTENING, EXECUTING, ERROR = Value
  }
  val state = RegInit(CmdFsmState.LISTENING)

  // Valid fields
  val addr_ab_valid = RegInit(false.B)
  val addr_cd_valid = RegInit(false.B)
  val size0_valid = RegInit(false.B)
  val size1_valid = RegInit(false.B)
  val config_ex_valid = RegInit(false.B)
  val bias_valid = RegInit(false.B)
  
  // Ready logic: if we're not actively executing, take new commands
  val ready = RegInit(true.B) 
  io.cmd.ready := ready
  
  // Initialization
  io.tiler.valid := false.B
  io.tiler.bits.dtype := 0.U
  io.tiler.bits.addr_a := 0.U 
  io.tiler.bits.addr_b := 0.U 
  io.tiler.bits.addr_c := 0.U 
  io.tiler.bits.addr_d := 0.U 
  io.tiler.bits.m := 0.U 
  io.tiler.bits.n := 0.U 
  io.tiler.bits.k := 0.U 
  io.tiler.bits.repeating_bias := false.B 
  io.tiler.bits.dataflow := false.B 
  io.tiler.bits.activation := 0.U 
  io.tiler.bits.acc_rshift := 0.U 
  io.tiler.bits.matmul_rshift := 0.U
  io.tiler.bits.relu6_lshift := 0.U

  def reset_and_listen(): Unit = {
    // Reset all data-validity
    addr_ab_valid := false.B
    addr_cd_valid := false.B
    size0_valid := false.B
    size1_valid := false.B
    config_ex_valid := false.B
    bias_valid := false.B
    // And go back to listening for commands
    state := CmdFsmState.LISTENING
  } 

  // FSM Behavior 
  when (state === CmdFsmState.EXECUTING) {
    // Pending EXECUTION ongoing 
    ready := false.B 
    io.tiler.valid := false.B 
    // Wait for tiling/ execution to complete, let any further commands queue up 
    when (io.tiler.fire()) {
      ready := true.B 
      state := CmdFsmState.LISTENING
    }
  }.elsewhen (state === CmdFsmState.ERROR) {
    // In ERROR state - only update based on RESET commands 
    io.tiler.valid := false.B
    when (io.cmd.fire()) {
      val cmd = io.cmd.bits
      val funct = cmd.inst.funct
      when (funct === RESET) {
        reset_and_listen()
      } // All other commands are ignored 
    }
  }.otherwise { // LISTENING State
    when (io.cmd.fire()) {
      val cmd = io.cmd.bits
      val funct = cmd.inst.funct
      val rs1 = cmd.rs1 
      val rs2 = cmd.rs2 
      // Execute command
      when (funct === COMPUTE_AND_FLIP_CMD) { 
        // Signal to the Tiler, and move to our EXEC state 
        // FIXME: check all valid 
        state := CmdFsmState.EXECUTING
        io.tiler.valid := true.B
      }
      .elsewhen (funct === CONFIG_CMD) {
        // FIXME: check validity of all these settings 
        io.tiler.bits.dataflow := rs1(2).asBool
        io.tiler.bits.activation := rs1(4,3)
        io.tiler.bits.acc_rshift := rs1(63,32)  // FIXME: triple check we dont swap these two shifts
        io.tiler.bits.matmul_rshift := rs2(31,0) 
        io.tiler.bits.relu6_lshift := rs2(63,32)
        config_ex_valid := true.B
      } 
      .elsewhen (funct === ADDR_AB) {
        io.tiler.bits.addr_a := rs1
        io.tiler.bits.addr_b := rs2
        addr_ab_valid := true.B
      } 
      .elsewhen (funct === ADDR_CD) {
        io.tiler.bits.addr_c := rs1
        io.tiler.bits.addr_d := rs2
        addr_cd_valid := true.B
      } 
      .elsewhen (funct === SIZE0) {
        io.tiler.bits.m := rs1
        io.tiler.bits.n := rs2
        size0_valid := true.B
      } 
      .elsewhen (funct === SIZE1) {
        io.tiler.bits.k := rs1
        size1_valid := true.B
      } 
      .elsewhen (funct === RPT_BIAS) {
        io.tiler.bits.repeating_bias := rs1(0).asBool
        bias_valid := true.B
      } 
      .elsewhen (funct === RESET) {
        reset_and_listen()
      } 
      .otherwise {
        // Invalid command type 
        // FIXME: error-cause setup
        state := CmdFsmState.ERROR
      }
    }
  }
}

class TilerFsm extends Module {
  // IO-Prototype for Tiler FSM
  val io = IO(new Bundle {
    val cmd = Flipped(Decoupled(new CmdTilerInterface))
    //val rob = TBD!;
  });
  io.cmd.ready := false.B;
}

class LocalAddr(sp_banks: Int, sp_bank_entries: Int, acc_banks: Int, acc_bank_entries: Int) extends Bundle {
  private val localAddrBits = 32 // TODO magic number

  private val spAddrBits = log2Ceil(sp_banks * sp_bank_entries)
  private val accAddrBits = log2Ceil(acc_banks * acc_bank_entries)
  private val maxAddrBits = spAddrBits max accAddrBits

  private val spBankBits = log2Up(sp_banks)
  private val spBankRowBits = log2Up(sp_bank_entries)

  private val accBankBits = log2Up(acc_banks)
  private val accBankRowBits = log2Up(acc_bank_entries)

  val is_acc_addr = Bool()
  val accumulate = Bool()
  val garbage = UInt((localAddrBits - maxAddrBits - 2).W)
  val data = UInt(maxAddrBits.W)

  def sp_bank(dummy: Int = 0) = if (spAddrBits == spBankRowBits) 0.U else data(spAddrBits - 1, spBankRowBits)
  def sp_row(dummy: Int = 0) = data(spBankRowBits - 1, 0)
  def acc_bank(dummy: Int = 0) = if (accAddrBits == accBankRowBits) 0.U else data(accAddrBits - 1, accBankRowBits)
  def acc_row(dummy: Int = 0) = data(accBankRowBits - 1, 0)

  def full_sp_addr(dummy: Int = 0) = data(spAddrBits - 1, 0)
  def full_acc_addr(dummy: Int = 0) = data(accAddrBits - 1, 0)

  def is_same_address(other: LocalAddr): Bool = is_acc_addr === other.is_acc_addr && data === other.data
  def is_same_address(other: UInt): Bool = is_same_address(other.asTypeOf(this))
  def is_garbage(dummy: Int = 0) = is_acc_addr && accumulate && data.andR() /* &&
    { if (garbage.getWidth > 0) garbage(0) else true.B } */

  def +(other: UInt) = {
    require(isPow2(sp_bank_entries)) // TODO remove this requirement
    require(isPow2(acc_bank_entries)) // TODO remove this requirement

    val result = WireInit(this)
    result.data := data + other
    result
  }

  def <=(other: LocalAddr) =
    is_acc_addr === other.is_acc_addr &&
      Mux(is_acc_addr, full_acc_addr() <= other.full_acc_addr(), full_sp_addr() <= other.full_sp_addr())

  def >(other: LocalAddr) =
    is_acc_addr === other.is_acc_addr &&
      Mux(is_acc_addr, full_acc_addr() > other.full_acc_addr(), full_sp_addr() > other.full_sp_addr())

  def make_this_garbage(dummy: Int = 0): Unit = {
    is_acc_addr := true.B
    accumulate := true.B
    data := ~(0.U(maxAddrBits.W))
  }

  override def cloneType: LocalAddr.this.type = new LocalAddr(sp_banks, sp_bank_entries, acc_banks, acc_bank_entries).asInstanceOf[this.type]
}

class Gemmini[T <: Data : Arithmetic](opcodes: OpcodeSet, val config: GemminiArrayConfig[T])
                                     (implicit p: Parameters)
  extends LazyRoCC (
    opcodes = OpcodeSet.custom3,
    nPTWPorts = 1) {

  Files.write(Paths.get(config.headerFilePath), config.generateHeader().getBytes(StandardCharsets.UTF_8))

  val xLen = p(XLen)
  val spad = LazyModule(new Scratchpad(config))

  override lazy val module = new GemminiModule(this)
  override val tlNode = spad.id_node
}

class GemminiModule[T <: Data: Arithmetic]
    (outer: Gemmini[T])
    extends LazyRoCCModuleImp(outer)
    with HasCoreParameters {

  import outer.config._
  import outer.spad

  val tagWidth = 32

  // Command & Tiler FSMs
  // FIXME: only hooked up to each other thus far 
  val cmd_fsm = Module(new CmdFsm)
  val tiler_fsm = Module(new TilerFsm)
  cmd_fsm.io.tiler <> tiler_fsm.io.cmd;

  // Incoming commands and ROB
  val raw_cmd = Queue(io.cmd)
  cmd_fsm.io.cmd <> raw_cmd;
  
  // TLB
  implicit val edge = outer.tlNode.edges.out.head
  val tlb = Module(new FrontendTLB(2, 4, dma_maxbytes))
  (tlb.io.clients zip outer.spad.module.io.tlb).foreach(t => t._1 <> t._2)
  tlb.io.exp.flush_skip := false.B
  tlb.io.exp.flush_retry := false.B

  dontTouch(outer.spad.module.io.tlb)

  io.ptw.head <> tlb.io.ptw
  /*io.ptw.head.req <> tlb.io.ptw.req
  tlb.io.ptw.resp <> io.ptw.head.resp
  tlb.io.ptw.ptbr := io.ptw.head.ptbr
  tlb.io.ptw.status := outer.spad.module.io.mstatus
  tlb.io.ptw.pmp := io.ptw.head.pmp
  tlb.io.ptw.customCSRs := io.ptw.head.customCSRs*/

  spad.module.io.flush := tlb.io.exp.flush()


  // val unrolled_cmd = LoopUnroller(raw_cmd, outer.config.meshRows * outer.config.tileRows)
  // // val compressed_cmd = InstCompressor(raw_cmd)
  // val compressed_cmd = InstCompressor(unrolled_cmd)
  // compressed_cmd.ready := false.B

  // val rob = Module(new ROB(new RoCCCommand, rob_entries, local_addr_t, meshRows*tileRows))
  // val cmd_decompressor = Module(new InstDecompressor(rob_entries))

  // cmd_decompressor.io.in.valid := rob.io.issue.ex.valid
  // cmd_decompressor.io.in.bits.cmd := rob.io.issue.ex.cmd
  // cmd_decompressor.io.in.bits.rob_id := rob.io.issue.ex.rob_id
  // rob.io.issue.ex.ready := cmd_decompressor.io.in.ready

  // val decompressed_cmd = cmd_decompressor.io.out

  // // Controllers
  // val load_controller = Module(new LoadController(outer.config, coreMaxAddrBits, local_addr_t))
  // val store_controller = Module(new StoreController(outer.config, coreMaxAddrBits, local_addr_t))
  // val ex_controller = Module(new ExecuteController(xLen, tagWidth, outer.config))

  // load_controller.io.cmd.valid := rob.io.issue.ld.valid
  // rob.io.issue.ld.ready := load_controller.io.cmd.ready
  // load_controller.io.cmd.bits.cmd := rob.io.issue.ld.cmd
  // load_controller.io.cmd.bits.cmd.inst.funct := rob.io.issue.ld.cmd.inst.funct
  // load_controller.io.cmd.bits.rob_id := rob.io.issue.ld.rob_id

  // store_controller.io.cmd.valid := rob.io.issue.st.valid
  // rob.io.issue.st.ready := store_controller.io.cmd.ready
  // store_controller.io.cmd.bits.cmd := rob.io.issue.st.cmd
  // store_controller.io.cmd.bits.cmd.inst.funct := rob.io.issue.st.cmd.inst.funct
  // store_controller.io.cmd.bits.rob_id := rob.io.issue.st.rob_id

////   ex_controller.io.cmd.valid := rob.io.issue.ex.valid
////   rob.io.issue.ex.ready := ex_controller.io.cmd.ready
////   ex_controller.io.cmd.bits.cmd := rob.io.issue.ex.cmd
////   ex_controller.io.cmd.bits.cmd.inst.funct := rob.io.issue.ex.cmd.inst.funct
////   ex_controller.io.cmd.bits.rob_id := rob.io.issue.ex.rob_id
  // ex_controller.io.cmd <> decompressed_cmd

  // // Wire up scratchpad to controllers
  // spad.module.io.dma.read <> load_controller.io.dma
  // spad.module.io.dma.write <> store_controller.io.dma
  // ex_controller.io.srams.read <> spad.module.io.srams.read
  // ex_controller.io.srams.write <> spad.module.io.srams.write
  // ex_controller.io.acc.read <> spad.module.io.acc.read
  // ex_controller.io.acc.write <> spad.module.io.acc.write

  // // Wire up controllers to ROB
  // rob.io.alloc.valid := false.B
  // rob.io.alloc.bits := compressed_cmd.bits

  // val rob_completed_arb = Module(new Arbiter(UInt(log2Up(rob_entries).W), 3))

  // rob_completed_arb.io.in(0).valid := ex_controller.io.completed.valid
  // rob_completed_arb.io.in(0).bits := ex_controller.io.completed.bits

  // rob_completed_arb.io.in(1) <> load_controller.io.completed
  // rob_completed_arb.io.in(2) <> store_controller.io.completed

  // rob.io.completed.valid := rob_completed_arb.io.out.valid
  // rob.io.completed.bits := rob_completed_arb.io.out.bits
  // rob_completed_arb.io.out.ready := true.B

  // // Wire up global RoCC signals
  // // io.busy := cmd.valid || load_controller.io.busy || store_controller.io.busy || spad.module.io.busy || rob.io.busy
  // io.busy := rob.io.busy || spad.module.io.busy
  // io.interrupt := tlb.io.exp.interrupt


  // Issue commands to controllers
  // TODO we combinationally couple cmd.ready and cmd.valid signals here
  // when (compressed_cmd.valid) {
  //   // val config_cmd_type = cmd.bits.rs1(1,0) // TODO magic numbers

  //   val funct = compressed_cmd.bits.inst.funct
  //   val is_flush = funct === FLUSH_CMD
  //   /*
  //   val is_load = (funct === LOAD_CMD) || (funct === CONFIG_CMD && config_cmd_type === CONFIG_LOAD)
  //   val is_store = (funct === STORE_CMD) || (funct === CONFIG_CMD && config_cmd_type === CONFIG_STORE)
  //   val is_ex = (funct === COMPUTE_AND_FLIP_CMD || funct === COMPUTE_AND_STAY_CMD || funct === PRELOAD_CMD) ||
  //   (funct === CONFIG_CMD && config_cmd_type === CONFIG_EX)
  //   */

  //   when (is_flush) {
  //     val skip = compressed_cmd.bits.rs1(0)
  //     tlb.io.exp.flush_skip := skip
  //     tlb.io.exp.flush_retry := !skip

  //     compressed_cmd.ready := true.B // TODO should we wait for an acknowledgement from the TLB?
  //   }

  //   .otherwise {
  //     rob.io.alloc.valid := true.B

  //     when(rob.io.alloc.fire()) {
  //       compressed_cmd.ready := true.B
  //     }
  //   }

  //   /*
  //   .elsewhen (is_load) {
  //     load_controller.io.cmd.valid := true.B

  //     when (load_controller.io.cmd.fire()) {
  //       cmd.ready := true.B
  //     }
  //   }

  //   .elsewhen (is_store) {
  //     store_controller.io.cmd.valid := true.B

  //     when (store_controller.io.cmd.fire()) {
  //       cmd.ready := true.B
  //     }
  //   }

  //   .otherwise {
  //     ex_controller.io.cmd.valid := true.B

  //     when (ex_controller.io.cmd.fire()) {
  //       cmd.ready := true.B
  //     }

  //     assert(is_ex, "unknown gemmini command")
  //   }
  //   */
  // }
}
