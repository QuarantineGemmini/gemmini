//============================================================================
// this file contains the top-level Gemmini2 controller
// - it uses a CmdController to parse the input RoCC commands
// - it uses a TilerController to drive tiled matmul in hardware
// - it has DMA engines that do im2col on the fly
//============================================================================
package gemmini

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths}

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import GemminiISA._

class Gemmini2[T <: Data : Arithmetic]
  (opcodes: OpcodeSet, val config: GemminiArrayConfig[T])
  (implicit p: Parameters)
  extends LazyRoCC(opcodes=OpcodeSet.custom3, nPTWPorts=1) {

  Files.write(Paths.get(config.headerFilePath),
              config.generateHeader().getBytes(StandardCharsets.UTF_8))

  override lazy val module = new GemminiModule2(this, config)

  // diplomatic scratchpad
  val spad = LazyModule(new Scratchpad(config))
  override val tlNode = spad.id_node
}

class GemminiModule2[T <: Data: Arithmetic]
  (outer: Gemmini2[T], config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends LazyRoCCModuleImp(outer) with HasCoreParameters {
  import outer.config._
  import outer.spad

  //=========================================================================
  // TLB (2-clients, 4 entries)
  //=========================================================================
  val tlb = FrontendTLB(2, 4, dma_maxbytes, outer.tlNode.edges.out.head)
  tlb.io.clients(0)    <> outer.spad.module.io.tlb(0)
  tlb.io.clients(1)    <> outer.spad.module.io.tlb(1)
  io.ptw.head          <> tlb.io.ptw
  io.interrupt         := tlb.io.exp.interrupt
  spad.module.io.flush := tlb.io.exp.flush()

  dontTouch(outer.spad.module.io.tlb)

  //=========================================================================
  // OoO Issuing
  //=========================================================================
  val raw_cmd = Queue(io.cmd)

  val cmd_fsm = CmdFSM(outer.config)
  cmd_fsm.io.cmd         <> raw_cmd
  tlb.io.exp.flush_retry := cmd_fsm.io.flush_retry
  tlb.io.exp.flush_skip  := cmd_fsm.io.flush_skip

  val tiler = TilerController(outer.config)
  tiler.io.cmd_in <> cmd_fsm.io.tiler

  //=========================================================================
  // OoO Execution
  //=========================================================================
  val exec = ExecuteController(outer.config)
  exec.io.cmd                <> tiler.io.issue.exec
  tiler.io.completed.exec    := exec.io.completed
  spad.module.io.srams.read  <> exec.io.srams.read
  spad.module.io.srams.write <> exec.io.srams.write
  spad.module.io.acc.read    <> exec.io.acc.read
  spad.module.io.acc.write   <> exec.io.acc.write

  val load = LoadController(outer.config)
  load.io.cmd             <> tiler.io.issue.load
  load.io.cfg             <> cmd_fsm.io.tiler 
  tiler.io.completed.load <> load.io.completed
  spad.module.io.dma.read <> load.io.dma

  val store = StoreController(outer.config)
  store.io.cmd             <> tiler.io.issue.store
  tiler.io.completed.store <> store.io.completed
  spad.module.io.dma.write <> store.io.dma

  //=========================================================================
  // Busy Signal (used by RocketCore during fence insn)
  //=========================================================================
  val is_computing = cmd_fsm.io.busy || tiler.io.busy ||
                     spad.module.io.busy ||
                     load.io.busy || store.io.busy || exec.io.busy

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

  exec.io.prof := prof

  when(prof.end) {
    printf(s"G2-PERF[%d]: total-cycles: %d\n", prof.debug_cycle, prof.cycle)
  }
}
