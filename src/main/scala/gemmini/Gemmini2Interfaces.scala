//============================================================================
// this contains interfaces between the Gemmini2 components
//============================================================================
package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import GemminiISA._

// Decoupled returns the writer, Flipped(Decoupled) returns the reader

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


//class TilerEvent(config: GemminiArrayConfig[T])(implicit p: Parameters)
//  extends Bundle with HasCoreParameters
//{
//  // TODO: enumerate types of events
//  // TODO: figure out better way to union the possible data for each event.
//  //       should we burst the data if it doesn't fit in 32 bits?
//  val valid = Output(Bool())
//  val type  = Output(UInt(5.W))
//  val data  = Output(Uint(32.W))
//
//  override def cloneType: this.type =
//    new TilerEvent(config).asInstanceOf[this.type]
//}

