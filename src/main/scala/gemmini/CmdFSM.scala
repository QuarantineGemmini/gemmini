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
    val tiler       = Decoupled(new TilerCmd(OTYPE_BITS_IDX))
    val flush_retry = Output(Bool())
    val flush_skip  = Output(Bool())
    val busy        = Output(Bool())
  });

  //==========================================================================
  // local state registers
  //==========================================================================
  val m              = RegInit(0.U(32.W))
  val n              = RegInit(0.U(32.W))
  val k              = RegInit(0.U(32.W))
  val addr_a         = RegInit(0.U(xLen.W))
  val addr_b         = RegInit(0.U(xLen.W))
  val addr_c         = RegInit(0.U(xLen.W))
  val addr_d         = RegInit(0.U(xLen.W))
  val addr_a_cfg     = RegInit(0.U.asTypeOf(new TilerCmdAddrCfg))
  val addr_b_cfg     = RegInit(0.U.asTypeOf(new TilerCmdAddrCfg))
  val addr_c_cfg     = RegInit(0.U.asTypeOf(new TilerCmdAddrCfg))
  val addr_d_cfg     = RegInit(0.U.asTypeOf(new TilerCmdAddrCfg))
  val in_rshift      = RegInit(0.U(OTYPE_BITS_IDX.W))
  val acc_rshift     = RegInit(0.U(OTYPE_BITS_IDX.W))
  val relu6_lshift   = RegInit(0.U(OTYPE_BITS_IDX.W))
  val activation     = RegInit(0.U(2.W))
  val repeating_bias = RegInit(0.U(1.W))

  // Valid fields
  val config_ex_valid = RegInit(false.B)
  val addr_ab_valid   = RegInit(false.B)
  val addr_cd_valid   = RegInit(false.B)
  val size0_valid     = RegInit(false.B)
  val size1_valid     = RegInit(false.B)
  val bias_valid      = RegInit(false.B)

  // pass CSR status bits to tiler
  // - TODO: fix this api. this works only if sw fence() is always used
  val status = Reg(new MStatus)
  status := DontCare

  //==========================================================================
  // Combinational Output Defaults 
  //==========================================================================
  io.cmd.ready   := false.B
  io.tiler.valid := false.B
  io.flush_retry := false.B
  io.flush_skip  := false.B

  io.tiler.bits.m              := m
  io.tiler.bits.n              := n
  io.tiler.bits.k              := k
  io.tiler.bits.addr_a         := addr_a
  io.tiler.bits.addr_b         := addr_b
  io.tiler.bits.addr_c         := addr_c
  io.tiler.bits.addr_d         := addr_d
  io.tiler.bits.addr_a_cfg     := addr_a_cfg 
  io.tiler.bits.addr_b_cfg     := addr_b_cfg
  io.tiler.bits.addr_c_cfg     := addr_c_cfg
  io.tiler.bits.addr_d_cfg     := addr_d_cfg
  io.tiler.bits.in_rshift      := in_rshift
  io.tiler.bits.acc_rshift     := acc_rshift
  io.tiler.bits.relu6_lshift   := relu6_lshift
  io.tiler.bits.activation     := activation
  io.tiler.bits.repeating_bias := repeating_bias
  io.tiler.bits.status         := status

  //==========================================================================
  // FSM 
  //==========================================================================
  val (s_LISTENING :: s_EX_PENDING Nil) = Enum(2)
  val state = RegInit(s_LISTENING)

  // gemmini implicitly resets all state after each compute() insn
  def reset_and_listen(check: Boolean): Unit = {
    if(check) {
      assert(config_ex_valid)
      assert(addr_ab_valid)
      assert(addr_cd_valid)
      assert(size0_valid)
      assert(size1_valid)
      assert(bias_valid)
    }
    config_ex_valid := false.B
    addr_ab_valid   := false.B
    addr_cd_valid   := false.B
    size0_valid     := false.B
    size1_valid     := false.B
    bias_valid      := false.B
    addr_a_cfg.mode := ADDR_MODE_NORMAL
    addr_b_cfg.mode := ADDR_MODE_NORMAL
    addr_c_cfg.mode := ADDR_MODE_NORMAL
    addr_d_cfg.mode := ADDR_MODE_NORMAL
    state           := s_LISTENING
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
        .elsewhen ((funct === CONFIG_CMD) && (rs1(1,0) === CONFIG_EX)) {
          assert(rs1(2) === WEIGHT_STATIONARY)
          acc_rshift      := rs1(63,32)
          activation      := rs1(4,3)
          relu6_lshift    := rs2(63,32)
          in_rshift       := rs2(31,0)
          config_ex_valid := true.B
          assert(!config_ex_valid)
        }
        .elsewhen (funct === ADDR_AB_CMD) {
          addr_a        := rs1
          addr_b        := rs2
          addr_ab_valid := true.B
          assert(!addr_ab_valid)
        }
        .elsewhen (funct === ADDR_CD_CMD) {
          addr_c        := rs1
          addr_d        := rs2
          addr_cd_valid := true.B
          assert(!addr_cd_valid)
        }
        .elsewhen (funct === SIZE0_CMD) {
          m           := rs1
          n           := rs2
          size0_valid := true.B
          assert(!size0_valid)
        }
        .elsewhen (funct === SIZE1_CMD) {
          k           := rs1
          size1_valid := true.B
          assert(!size1_valid)
        }
        .elsewhen (funct === RPT_BIAS_CMD) {
          repeating_bias := rs1(0).asBool
          bias_valid     := true.B
          assert(!bias_valid)
        }
        .elsewhen (funct === CONFIG_ADDR_MODE_A_CMD) {
          addr_a_cfg.in_rows     := rs1(63,32)
          addr_a_cfg.in_cols     := rs1(31,0)
          addr_a_cfg.stride      := rs2(63,48)
          addr_a_cfg.padding     := rs2(47,40)
          addr_a_cfg.in_channels := rs2(39,24)
          addr_a_cfg.kernel_size := rs2(23,8)
          addr_a_cfg.mode        := rs2(0)
        }
        .elsewhen (funct === CONFIG_ADDR_MODE_B_CMD) {
          addr_b_cfg.in_rows     := rs1(63,32)
          addr_b_cfg.in_cols     := rs1(31,0)
          addr_b_cfg.stride      := rs2(63,48)
          addr_b_cfg.padding     := rs2(47,40)
          addr_b_cfg.in_channels := rs2(39,24)
          addr_b_cfg.kernel_size := rs2(23,8)
          addr_b_cfg.mode        := rs2(0)
        }
        .elsewhen (funct === CONFIG_ADDR_MODE_C_CMD) {
          addr_c_cfg.in_rows     := rs1(63,32)
          addr_c_cfg.in_cols     := rs1(31,0)
          addr_c_cfg.stride      := rs2(63,48)
          addr_c_cfg.padding     := rs2(47,40)
          addr_c_cfg.in_channels := rs2(39,24)
          addr_c_cfg.kernel_size := rs2(23,8)
          addr_c_cfg.mode        := rs2(0)
        }
        .elsewhen (funct === CONFIG_ADDR_MODE_D_CMD) {
          addr_d_cfg.in_rows     := rs1(63,32)
          addr_d_cfg.in_cols     := rs1(31,0)
          addr_d_cfg.stride      := rs2(63,48)
          addr_d_cfg.padding     := rs2(47,40)
          addr_d_cfg.in_channels := rs2(39,24)
          addr_d_cfg.kernel_size := rs2(23,8)
          addr_d_cfg.mode        := rs2(0)
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
  io.busy := (state === s_EX_PENDING)
}

object CmdFSM {
  def apply(OTYPE_BITS_IDX: Int)(implicit p: Parameters) 
    = Module(new CmdFSM(OTYPE_BITS_IDX))
}
