//===========================================================================
// Command FSM
// * Receive RoCC commands
// * Check validity of configuration data
// * Hand off to Tiling on COMPUTE requests
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._
import freechips.rocketchip.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import GemminiISA._

class CmdFSM(OTYPE_BITS_IDX: Int)
  (implicit val p: Parameters) extends CoreModule {
  //==========================================================================
  // module ports
  //==========================================================================
  val io = IO(new Bundle {
    val cmd         = Flipped(Decoupled(new RoCCCommand))
    val tiler       = Decoupled(Bool())
    val flush_retry = Output(Bool())
    val flush_skip  = Output(Bool())
    val busy        = Output(Bool())
    val csr         = Output(new GemminiCSR(OTYPE_BITS_IDX))
  });
  io.tiler.bits := DontCare

  //==========================================================================
  // local state registers
  //==========================================================================
  val csr = RegInit(0.U.asTypeOf(new GemminiCSR))
  io.csr := csr

  // Valid fields
  val config_dp_valid  = RegInit(false.B)
  val mnk_valid        = RegInit(false.B)
  val addr_ab_valid    = RegInit(false.B)
  val addr_cd_valid    = RegInit(false.B)
  val addr_a_cfg_valid = RegInit(false.B)
  val addr_b_cfg_valid = RegInit(false.B)
  val addr_c_cfg_valid = RegInit(false.B)
  val addr_d_cfg_valid = RegInit(false.B)

  //==========================================================================
  // Combinational Output Defaults 
  //==========================================================================
  io.cmd.ready   := false.B
  io.tiler.valid := false.B
  io.flush_retry := false.B
  io.flush_skip  := false.B

  //==========================================================================
  // FSM 
  //==========================================================================
  val (s_LISTENING :: s_EX_PENDING Nil) = Enum(2)
  val state = RegInit(s_LISTENING)

  // gemmini implicitly resets all state after each compute() insn
  def reset_and_listen(check: Boolean): Unit = {
    if(check) {
      assert(config_dp_valid)
      assert(addr_ab_valid)
      assert(addr_cd_valid)
      assert(mnk_valid)
      assert(addr_a_cfg_valid)
      assert(addr_b_cfg_valid)
      assert(addr_c_cfg_valid)
      assert(addr_d_cfg_valid)
    }
    config_dp_valid  := false.B
    mnk_valid        := false.B
    addr_ab_valid    := false.B
    addr_cd_valid    := false.B
    addr_a_cfg_valid := false.B
    addr_b_cfg_valid := false.B
    addr_c_cfg_valid := false.B
    addr_d_cfg_valid := false.B
    state := s_LISTENING
  }

  switch (state) {
    is (s_EX_PENDING) {
      io.tiler.valid := true.B
      when (io.tiler.fire()) {
        reset_and_listen(true)
      }
    }
    is (s_LISTENING) {
      io.cmd.ready := true.B
      when (io.cmd.fire()) {
        val cmd   = io.cmd.bits
        val funct = cmd.inst.funct
        val rs1   = cmd.rs1
        val rs2   = cmd.rs2

        // always update status bits on a successful RoCC command
        status := cmd.status

        when (funct === FLUSH_CMD) {
          val skip = cmd.rs1(0)
          io.flush_retry := !skip
          io.flush_skip  := skip
        }
        .elsewhen (funct === COMPUTE_CMD) {
          io.tiler.valid := true.B
          when (io.tiler.fire()) {
            reset_and_listen(true)
          } .otherwise {
            state := s_EX_PENDING
          }
        }
        .elsewhen (funct === RESET_CMD) {
          reset_and_listen(false)
        }
        .elsewhen (funct === CONFIG_DATAPATH_CMD) {
          csr.acc_rshift      := rs1(63,32)
          csr.repeating_bias  := rs1(5)
          csr.activation      := rs1(4,3)
          csr.relu6_lshift    := rs2(63,32)
          config_dp_valid := true.B
          assert(!config_dp_valid)
        }
        .elsewhen (funct === MNK_CMD) {
          csr.m := rs1(63,32)
          csr.n := rs1(31,0)
          csr.k := rs2(31,0)
          mnk_valid := true.B
          assert(!mnk_valid)
        }
        .elsewhen (funct === ADDR_AB_CMD) {
          csr.addr_a_cfg.vaddr := rs1
          csr.addr_b_cfg.vaddr := rs2
          addr_ab_valid := true.B
          assert(!addr_ab_valid)
        }
        .elsewhen (funct === ADDR_CD_CMD) {
          csr.addr_c_cfg.vaddr := rs1
          csr.addr_d_cfg.vaddr := rs2
          addr_cd_valid := true.B
          assert(!addr_cd_valid)
        }
        .elsewhen (funct === CONFIG_ADDR_CMD) {
          val addr_cfg = WireInit(0.U.asTypeOf(new GemminiCSRAddr))
          val matrix_id = rs2(2,1)
          addr_cfg.mode := rs2(0)
          when (addr_cfg.mode === ADDR_MODE_NORMAL) {
            addr_cfg.normal_stride := rs1
          } .otherwise {
            addr_cfg.in_rows     := rs1(63,32)
            addr_cfg.in_cols     := rs1(31,0)
            addr_cfg.stride      := rs2(63,48)
            addr_cfg.padding     := rs2(47,40)
            addr_cfg.in_channels := rs2(39,24)
            addr_cfg.kernel_size := rs2(23,8)
          }
          // leave the base vaddr as is
          switch (matrix_id) {
            is (MATRIX_ID_A) { 
              csr.addr_a_cfg       := addr_cfg
              csr.addr_a_cfg.vaddr := csr.addr_a_cfg.vaddr
              addr_a_cfg_valid := true.B
              assert(!addr_a_cfg_valid)
            }
            is (MATRIX_ID_B) { 
              csr.addr_b_cfg       := addr_cfg
              csr.addr_b_cfg.vaddr := csr.addr_b_cfg.vaddr
              addr_b_cfg_valid := true.B
              assert(!addr_b_cfg_valid)
            }
            is (MATRIX_ID_C) { 
              csr.addr_c_cfg       := addr_cfg
              csr.addr_c_cfg.vaddr := csr.addr_c_cfg.vaddr
              addr_c_cfg_valid := true.B
              assert(!addr_c_cfg_valid)
            }
            is (MATRIX_ID_D) { 
              csr.addr_d_cfg       := addr_cfg
              csr.addr_d_cfg.vaddr := csr.addr_d_cfg.vaddr
              addr_d_cfg_valid := true.B
              assert(!addr_d_cfg_valid)
            }
          }
        }
        .otherwise {
          assert(false.B, "invalid cmd received!");
        }
      }
    }
  }
  //==========================================================================
  // we block if we have accepted the COMPUTE_CMD, but the tiler has
  // not started executing it yet.
  //==========================================================================
  io.busy := config_dp_valid || addr_ab_valid || addr_cd_valid || mnk_valid ||
             addr_a_cfg_valid || addr_b_cfg_valid ||
             addr_c_cfg_valid || addr_d_cfg_valid
}

object CmdFSM {
  def apply(OTYPE_BITS_IDX: Int)(implicit p: Parameters) 
    = Module(new CmdFSM(OTYPE_BITS_IDX))
}
