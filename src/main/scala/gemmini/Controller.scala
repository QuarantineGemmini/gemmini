package gemmini

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths}

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import GemminiISA._

class Gemmini[T <: Data : Arithmetic]
  (opcodes: OpcodeSet, val config: GemminiArrayConfig[T])
  (implicit p: Parameters)
  extends LazyRoCC (opcodes = OpcodeSet.custom3, nPTWPorts = 1) {

  Files.write(Paths.get(config.headerFilePath), 
              config.generateHeader().getBytes(StandardCharsets.UTF_8))

  val xLen = p(XLen)
  val spad = LazyModule(new Scratchpad(config))

  override lazy val module = new GemminiModule(this)
  override val tlNode = spad.id_node
}

class GemminiModule[T <: Data: Arithmetic](outer: Gemmini[T])
  extends LazyRoCCModuleImp(outer) with HasCoreParameters {
  import outer.config._
  import outer.spad

  // TLB
  val tlb = FrontendTLB(2, 4, dma_maxbytes, outer.tlNode.edges.out.head)
  (tlb.io.clients zip outer.spad.module.io.tlb).foreach(t => t._1 <> t._2)
  tlb.io.exp.flush_skip := false.B
  tlb.io.exp.flush_retry := false.B
  io.interrupt := tlb.io.exp.interrupt

  dontTouch(outer.spad.module.io.tlb)

  io.ptw.head <> tlb.io.ptw

  spad.module.io.flush := tlb.io.exp.flush()

  // Incoming commands and ROB
  val raw_cmd = Queue(io.cmd)
  val unrolled_cmd = LoopUnroller(raw_cmd, DIM)
  unrolled_cmd.ready := false.B

  val rob = Module(new ROB(rob_entries, DIM, DIM))

  // Controllers
  val load_controller = LoadController(outer.config)
  val store_controller = StoreController(outer.config)
  val ex_controller = ExecuteController(outer.config)

  load_controller.io.cmd  <> rob.io.issue.ld
  store_controller.io.cmd <> rob.io.issue.st
  ex_controller.io.cmd    <> rob.io.issue.ex

  // Wire up scratchpad to controllers
  spad.module.io.dma.read    <> load_controller.io.dma
  spad.module.io.dma.write   <> store_controller.io.dma
  spad.module.io.srams.read  <> ex_controller.io.srams.read
  spad.module.io.srams.write <> ex_controller.io.srams.write
  spad.module.io.acc.read    <> ex_controller.io.acc.read
  spad.module.io.acc.write   <> ex_controller.io.acc.write

  // Wire up controllers to ROB
  rob.io.alloc.valid := false.B
  // rob.io.alloc.bits := compressed_cmd.bits
  rob.io.alloc.bits := unrolled_cmd.bits

  val rob_completed_arb = Module(new Arbiter(UInt(log2Up(rob_entries).W), 3))
  rob_completed_arb.io.in(0).valid := ex_controller.io.completed.valid
  rob_completed_arb.io.in(0).bits  := ex_controller.io.completed.bits
  rob_completed_arb.io.in(1)       <> load_controller.io.completed
  rob_completed_arb.io.in(2)       <> store_controller.io.completed
  rob.io.completed                 <> rob_completed_arb.io.out

  // Issue commands to controllers
  when (unrolled_cmd.valid) {
    val funct = unrolled_cmd.bits.inst.funct
    val is_flush = funct === FLUSH_CMD
    when (is_flush) {
      val skip = unrolled_cmd.bits.rs1(0)
      tlb.io.exp.flush_skip := skip
      tlb.io.exp.flush_retry := !skip
      unrolled_cmd.ready := true.B
    }
    .otherwise {
      rob.io.alloc.valid := true.B
      when(rob.io.alloc.fire()) {
        unrolled_cmd.ready := true.B
      }
    }
  }

  //=========================================================================
  // Busy Signal (used by RocketCore during fence insn)
  //=========================================================================
  val is_computing = unrolled_cmd.valid || 
                     rob.io.busy ||
                     spad.module.io.busy ||
                     load_controller.io.busy || 
                     store_controller.io.busy || 
                     ex_controller.io.busy
  io.busy := raw_cmd.valid || is_computing 

  //=========================================================================
  // instrumentation for profiling counters
  // - don't use raw_cmd.valid, since it causes spurious prof_starts when
  //   the cmd_fsm is being configured
  //=========================================================================
  val busy_last  = RegNext(is_computing)
  val prof_start = WireDefault(~busy_last & is_computing)
  val prof_end   = WireDefault(busy_last & ~is_computing)

  val prof_cycle = RegInit(0.U(32.W))
  prof_cycle := Mux(prof_start, 0.U, prof_cycle + 1.U)

  val debug_cycle = RegInit(0.U(40.W))
  debug_cycle := debug_cycle + 1.U

  val prof = Wire(new Profiling)
  prof.start       := prof_start
  prof.end         := prof_end
  prof.cycle       := prof_cycle
  prof.debug_cycle := debug_cycle

  ex_controller.io.prof := prof

  //when(prof.end) {
  //  printf(s"G2-PERF[%d]: total-cycles: %d\n", prof.debug_cycle, prof.cycle)
  //}

}
