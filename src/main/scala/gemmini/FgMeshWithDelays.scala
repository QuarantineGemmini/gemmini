package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import gemmini.Util._

class FgMeshWithDelays[T <: Data:Arithmetic](config: FgGemminiArrayConfig[T])
  (implicit val p: Parameters) extends Module with HasCoreParameters {
  import config._
  //=========================================================================
  // module interface
  //=========================================================================
  val io = IO(new Bundle {
    val in_valid  = Input(Bool())
    val a         = Input(Vec(FG_DIM, inputType))
    val b         = Input(Vec(FG_DIM, inputType))
    val flipped   = Input(Bool())
    val out_valid = Output(Bool())
    val out       = Output(Vec(FG_DIM, accType))
  })
}
