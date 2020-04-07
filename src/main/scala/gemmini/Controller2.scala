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
  val valid_to_tiler = RegInit(false.B) 
  io.tiler.valid := valid_to_tiler
  val tiler = RegInit(0.U.asTypeOf(new CmdTilerInterface))
  io.tiler.bits <> tiler
  
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
    valid_to_tiler := false.B 
    // Wait for tiling/ execution to complete, let any further commands queue up 
    when (io.tiler.fire()) {
        printf("CmdFsm EXECUTION_FINISHED\n") 
      ready := true.B 
      state := CmdFsmState.LISTENING
    }
  }.elsewhen (state === CmdFsmState.ERROR) {
        printf("CmdFsm IN ERROR STATE\n") 
    // In ERROR state - only update based on RESET commands 
    valid_to_tiler := false.B
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
        printf("CmdFsm COMPUTE\n") 
        printf("CmdFsm rs1[%d] rs2[%d]\n", rs1, rs2) 
        // Signal to the Tiler, and move to our EXEC state 
        // FIXME: check all valid 
      printf(
        "CmdFsm STATE: a[%d], b[%d], c[%d], d[%d], m[%d], n[%d], k[%d]\n", 
          tiler.addr_a, tiler.addr_b, tiler.addr_c, tiler.addr_d,
          tiler.m, tiler.n, tiler.k
        ) 
        state := CmdFsmState.EXECUTING
        valid_to_tiler := true.B
      }
      .elsewhen (funct === CONFIG_CMD) {
        printf("CmdFsm CONFIG\n") 
        printf("CmdFsm rs1[%d] rs2[%d]\n", rs1, rs2) 
        // FIXME: check validity of all these settings 
        tiler.dataflow := rs1(2).asBool
        tiler.activation := rs1(4,3)
        tiler.acc_rshift := rs1(63,32)  // FIXME: triple check we dont swap these two shifts
        tiler.matmul_rshift := rs2(31,0) 
        tiler.relu6_lshift := rs2(63,32)
        config_ex_valid := true.B
      } 
      .elsewhen (funct === ADDR_AB) {
        printf("CmdFsm ADDR_AB\n") 
        printf("CmdFsm rs1[%d] rs2[%d]\n", rs1, rs2) 
        tiler.addr_a := rs1
        tiler.addr_b := rs2
        addr_ab_valid := true.B
      } 
      .elsewhen (funct === ADDR_CD) {
        printf("CmdFsm ADDR_CD\n") 
        printf("CmdFsm rs1[%d] rs2[%d]\n", rs1, rs2) 
        tiler.addr_c := rs1
        tiler.addr_d := rs2
        addr_cd_valid := true.B
      } 
      .elsewhen (funct === SIZE0) {
        printf("CmdFsm ADDR_SIZE0\n") 
        printf("CmdFsm rs1[%d] rs2[%d]\n", rs1, rs2) 
        tiler.m := rs1
        tiler.n := rs2
        size0_valid := true.B
      } 
      .elsewhen (funct === SIZE1) {
        printf("CmdFsm ADDR_SIZE1\n") 
        printf("CmdFsm rs1[%d] rs2[%d]\n", rs1, rs2) 
        tiler.k := rs1
        size1_valid := true.B
      } 
      .elsewhen (funct === RPT_BIAS) {
        printf("CmdFsm RPT_BIAS\n") 
        printf("CmdFsm rs1[%d] rs2[%d]\n", rs1, rs2) 
        tiler.repeating_bias := rs1(0).asBool
        bias_valid := true.B
      } 
      .elsewhen (funct === RESET) {
        printf("CmdFsm RESET\n") 
        reset_and_listen()
      } 
      .elsewhen (funct === FLUSH_CMD) {
        printf("CmdFsm FLUSH_CMD will be ignored\n") 
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
  val ready = RegInit(false.B)
  io.cmd.ready := ready 
  
  when (io.cmd.fire()) {
      printf(
        "CmdFsm TILER_EXECUTING: a[%d], b[%d], c[%d], d[%d], m[%d], n[%d], k[%d]\n", 
          io.cmd.bits.addr_a, io.cmd.bits.addr_b, io.cmd.bits.addr_c, io.cmd.bits.addr_d,
          io.cmd.bits.m, io.cmd.bits.n, io.cmd.bits.k
      )
    ready := true.B
  }
}


class Gemmini2[T <: Data : Arithmetic](opcodes: OpcodeSet, val config: GemminiArrayConfig[T])
                                     (implicit p: Parameters)
  extends LazyRoCC (
    opcodes = OpcodeSet.custom3,
    nPTWPorts = 1) {

  Files.write(Paths.get(config.headerFilePath), config.generateHeader().getBytes(StandardCharsets.UTF_8))

  val xLen = p(XLen)
  val spad = LazyModule(new Scratchpad(config))

  override lazy val module = new Gemmini2Module(this)
  override val tlNode = spad.id_node
}

class Gemmini2Module[T <: Data: Arithmetic]
    (outer: Gemmini2[T])
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
