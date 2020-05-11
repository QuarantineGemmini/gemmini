// See README.md for license details.
package gemmini

import chisel3._
import chisel3.util._

class PEControl[T <: Data : Arithmetic](accType: T) extends Bundle {
  // Which register should be propagated (and which should be accumulated)?
  val propagate = UInt(1.W) 
  override def cloneType: PEControl.this.type 
    = new PEControl(accType).asInstanceOf[this.type]
}

// A PE implementing a MAC operation. Configured as fully combinational when 
// integrated into a Mesh.
// @param width Data width of operands
class PE[T <: Data](inputType: T, outputType: T, accType: T)
                   (implicit ev: Arithmetic[T]) extends Module {
  import ev._

  val io = IO(new Bundle {
    val in_a  = Input(inputType)
    val in_b  = Input(outputType)
    val in_d  = Input(outputType)
    val out_a = Output(inputType)
    val out_b = Output(outputType)
    val out_c = Output(outputType)

    val in_ctrl = Input(new PEControl(accType))
    val out_ctrl = Output(new PEControl(accType))

    val in_valid = Input(Bool())
    val out_valid = Output(Bool())
  })

  val c1 = Reg(accType)
  val c2 = Reg(accType)

  val a     = io.in_a
  val b     = io.in_b
  val d     = io.in_d
  val prop  = io.in_ctrl.propagate
  val valid = io.in_valid

  io.out_a := a
  io.out_ctrl.propagate := prop
  io.out_valid := valid

  // hardcoded to WS
  when(prop.asBool) {
    io.out_c := c1
    io.out_b := b.mac(a, c2.withWidthOf(inputType))
    c1 := Mux(valid, d, c1)
  }
  .otherwise {
    io.out_c := c2
    io.out_b := b.mac(a, c1.withWidthOf(inputType))
    c2 := Mux(valid, d, c2)
  }
}
