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

  dontTouch(outer.spad.module.io.tlb)

  io.ptw.head <> tlb.io.ptw

  spad.module.io.flush := tlb.io.exp.flush()

  // Incoming commands and ROB
  val raw_cmd = Queue(io.cmd)
  val unrolled_cmd = LoopUnroller(raw_cmd, outer.config.meshRows * outer.config.tileRows)
  unrolled_cmd.ready := false.B

  val rob = Module(new ROB(new RoCCCommand, rob_entries, local_addr_t, meshRows*tileRows, meshColumns*tileColumns))

  // Controllers
  val load_controller = LoadController(outer.config)
  val store_controller = StoreController(outer.config)
  val ex_controller = ExecuteController(outer.config)

  load_controller.io.cmd.valid := rob.io.issue.ld.valid
  rob.io.issue.ld.ready := load_controller.io.cmd.ready
  load_controller.io.cmd.bits.cmd := rob.io.issue.ld.cmd
  load_controller.io.cmd.bits.cmd.inst.funct := rob.io.issue.ld.cmd.inst.funct
  load_controller.io.cmd.bits.rob_id := rob.io.issue.ld.rob_id

  store_controller.io.cmd.valid := rob.io.issue.st.valid
  rob.io.issue.st.ready := store_controller.io.cmd.ready
  store_controller.io.cmd.bits.cmd := rob.io.issue.st.cmd
  store_controller.io.cmd.bits.cmd.inst.funct := rob.io.issue.st.cmd.inst.funct
  store_controller.io.cmd.bits.rob_id := rob.io.issue.st.rob_id

  ex_controller.io.cmd.valid := rob.io.issue.ex.valid
  rob.io.issue.ex.ready := ex_controller.io.cmd.ready
  ex_controller.io.cmd.bits.cmd := rob.io.issue.ex.cmd
  ex_controller.io.cmd.bits.cmd.inst.funct := rob.io.issue.ex.cmd.inst.funct
  ex_controller.io.cmd.bits.rob_id := rob.io.issue.ex.rob_id
  // ex_controller.io.cmd <> decompressed_cmd

  // Wire up scratchpad to controllers
  spad.module.io.dma.read <> load_controller.io.dma
  spad.module.io.dma.write <> store_controller.io.dma
  ex_controller.io.srams.read <> spad.module.io.srams.read
  ex_controller.io.srams.write <> spad.module.io.srams.write
  ex_controller.io.acc.read <> spad.module.io.acc.read
  ex_controller.io.acc.write <> spad.module.io.acc.write

  // Wire up controllers to ROB
  rob.io.alloc.valid := false.B
  // rob.io.alloc.bits := compressed_cmd.bits
  rob.io.alloc.bits := unrolled_cmd.bits

  val rob_completed_arb = Module(new Arbiter(UInt(log2Up(rob_entries).W), 3))

  rob_completed_arb.io.in(0).valid := ex_controller.io.completed.valid
  rob_completed_arb.io.in(0).bits := ex_controller.io.completed.bits

  rob_completed_arb.io.in(1) <> load_controller.io.completed
  rob_completed_arb.io.in(2) <> store_controller.io.completed

  rob.io.completed.valid := rob_completed_arb.io.out.valid
  rob.io.completed.bits := rob_completed_arb.io.out.bits
  rob_completed_arb.io.out.ready := true.B

  io.busy := raw_cmd.valid || unrolled_cmd.valid || 
             rob.io.busy || spad.module.io.busy
  io.interrupt := tlb.io.exp.interrupt

  // Issue commands to controllers
  when (unrolled_cmd.valid) {
    val funct = unrolled_cmd.bits.inst.funct

    val is_flush = funct === FLUSH_CMD

    when (is_flush) {
      // val skip = compressed_cmd.bits.rs1(0)
      val skip = unrolled_cmd.bits.rs1(0)
      tlb.io.exp.flush_skip := skip
      tlb.io.exp.flush_retry := !skip

      unrolled_cmd.ready := true.B
    }

    .otherwise {
      rob.io.alloc.valid := true.B

      when(rob.io.alloc.fire()) {
        // compressed_cmd.ready := true.B
        unrolled_cmd.ready := true.B
      }
    }
  }
}
