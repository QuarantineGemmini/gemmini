package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._

// WARNING: the hardware is hardcoded to assume the matrix dimensions are
//          all less than 2^32 bytes in each direction

//===========================================================================
// TilerController Interface
//===========================================================================
class TilerCmd(config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends Bundle with HasCoreParameters
{
  import config._
  val M              = Input(UInt(32.U))
  val N              = Input(UInt(32.U))
  val K              = Input(UInt(32.U))
  val A              = Input(UInt(xLen.W))
  val B              = Input(UInt(xLen.W))
  val D              = Input(UInt(xLen.W))
  val C              = Input(UInt(xLen.W))
  val in_shift       = Input(UInt(log2Up(accType.getWidth).W))
  val acc_shift      = Input(UInt(log2Up(accType.getWidth).W))
  val relu6_shift    = Input(UInt(log2Up(accType.getWidth).W))
  val activation     = Input(UInt(2.W))
  val repeating_bias = Input(Bool())

  override def cloneType: this.type =
    new TilerCmd(config).asInstanceOf[this.type]
}

class TilerEvent(config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends Bundle with HasCoreParameters
{
  // TODO: enumerate types of events
  // TODO: figure out better way to union the possible data for each event.
  //       should we burst the data if it doesn't fit in 32 bits?
  val valid = Output(Bool())
  val type  = Output(UInt(5.W))
  val data  = Output(Uint(32.W))

  override def cloneType: this.type =
    new TilerEvent(config).asInstanceOf[this.type]
}

object TilerSchedulerIO {
    val completed = Flipped(Valid(UInt(log2Up(rob_entries).W)))
  }

object TilerControllerIO {
  def apply(config: GemminiArrayConfig[T]) = IO(new Bundle {

    val in = Flipped(Decoupled(new TilerCmd))

    val issue = new Bundle {
      val load  = new ROBIssue(cmd_t, rob_entries)
      val store = new ROBIssue(cmd_t, rob_entries)
      val exec  = new ROBIssue(cmd_t, rob_entries)
      val flush = new ROBIssue(cmd_t, rob_entries)
    }
    val completed = new Bundle {
      val exec  = Flipped(Valid(UInt(log2Up(rob_entries).W)))
      val load  = Flipped(Decoupled(UInt(log2Up(rob_entries).W)))
      val store = Flipped(Decoupled(UInt(log2Up(rob_entries).W)))
      // TODO: handle flush at CmdParser???
      //val flush = Flipped(Decoupled(UInt(log2Up(rob_entries).W)))
    }
    val busy = Output(Bool())

    val eventbus =
  }
}

class RoCCInstruction extends Bundle {
  val funct = Bits(width = 7)
  val rs2 = Bits(width = 5)
  val rs1 = Bits(width = 5)
  val xd = Bool()
  val xs1 = Bool()
  val xs2 = Bool()
  val rd = Bits(width = 5)
  val opcode = Bits(width = 7)
}

class RoCCCommand(implicit p: Parameters) extends CoreBundle()(p) {
  val inst = new RoCCInstruction
  val rs1 = Bits(width = xLen)
  val rs2 = Bits(width = xLen)
  val status = new MStatus
}

val sched_insn1 = new RoCCInstruction
val sched_insn2 = new RoCCInstruction
val sched_cmd1 = new RoCCommand
val sched_cmd2 = new RoCCommand

def make_mvin_cmd()(implicit p: Parameters) = {
  sched_cmd1.inst.funct := CONFIG_CMD
  sched_cmd1.rs1        := CONFIG_LOAD
  sched_cmd1.rs2        :=
  sched_cmd2.inst.funct := LOAD_CMD

class TilerSchedulerCmd()

class TilerEvent(config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends Bundle with HasCoreParameters
{
  // TODO: enumerate types of events
  // TODO: figure out better way to union the possible data for each event.
  //       should we burst the data if it doesn't fit in 32 bits?
  val valid = Output(Bool())
  val type  = Output(UInt(5.W))
  val data  = Output(Uint(32.W))

  override def cloneType: this.type =
    new TilerEvent(config).asInstanceOf[this.type]
}

//===========================================================================
// TilerController Implementation
//===========================================================================
class TilerController(config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends Bundle with HasCoreParameters
{
  import config._

  val io = TilerControllerIO(config)

  //=========================================================================
  // arbitrate incoming completions
  //=========================================================================
  val completed_arb = Module(new Arbiter(UInt(log2Up(rob_entries).W), 4))
  completed_arb.io.in(0).valid := io.completed.exec.io.completed.valid
  completed_arb.io.in(0).bits  := io.completed.exec.io.completed.bits
  completed_arb.io.in(1)       <> io.completed.load.io.completed
  completed_arb.io.in(2)       <> io.completed.store.io.completed
  completed_arb.io.in(3)       <> io.completed.flush.io.completed

  var fsm = TilerFSM(config)
  fsm.io.cmd_in <> io.cmd_in

  var sched = TilerScheduler(config)
  sched.io.cmd_in <> fsm.io.sched_out
  sched.

  f
  // NOTE: i have to emit RoCCcommands to each unit!

  //val completed_valid        = completed_arb.io.out.valid
  //val completed_tag          = completed_arb.io.out.bits
  //completed_arb.io.out.ready := retire_completed_command

}

// Decoupled returns the writer, Flipped(Decoupled) returns the reader


object TilerFSM {
  def apply(enq: ReadyValidIO[RoCCCommand], block_size: Int)(implicit p: Parameters): DecoupledIO[RoCCCommand] = {
  //  val lu = Module(new LoopUnroller(block_size))
  //  lu.io.in <> enq
  //  lu.io.out
  //}
}
