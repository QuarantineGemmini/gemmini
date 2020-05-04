package gemmini

import chisel3._
import chisel3.util._

class FgPEControl extends Bundle {
  val prop = Bool()
}

class FgPE[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit ev: Arithmetic[T]) extends Module {
  import config._
  import ev._
  //=========================================================================
  // module interface
  //=========================================================================
  val io = IO(new Bundle {
    val in_a  = Input(inputType)
    val in_b  = Input(outputType)
    val in_d  = Input(outputType)
    val out_a = Output(inputType)
    val out_b = Output(outputType)
    val out_c = Output(outputType)

    val in_ctrl = Input(new FgPEControl)
    val out_ctrl = Output(new FgPEControl)

    val in_valid = Input(Bool())
    val out_valid = Output(Bool())
  })

  val b1 = Reg(inputType)
  val b2 = Reg(inputType)

  val a     = io.in_a
  val b     = io.in_b
  val d     = io.in_d
  val prop  = io.in_ctrl.prop
  val valid = io.in_valid

  io.out_a         := a
  io.out_ctrl.prop := prop
  io.out_valid     := valid

  when(prop.asBool) {
    io.out_b := b1
    io.out_c := d.mac(a, b2.withWidthOf(inputType))
    b1 := Mux(valid, b, b1)
  } .otherwise {
    io.out_b := b2
    io.out_c := d.mac(a, b1.withWidthOf(inputType))
    b2 := Mux(valid, b, b2)
  }
}
