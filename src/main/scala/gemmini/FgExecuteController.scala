//============================================================================
// Fine-grained-systolic-array exec-controller
// (new parameter: fg_sa_div)
//
// WARNING!!! the 'b-addr' and 'd-addr' within this ExecuteController and
// all submodules are swapped with the meaning of 'b-addr' and 'd-addr' 
// in all external modules! this was due to originally supporting WS and OS
// modes simultaneously. Just remember that d-addr within this module means
// the weights that is preloaded into the matrix, and the 'b-addr' is the
// address that is preloaded into the accumulator (so it isn't used here!)
//============================================================================
package gemmini

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths}

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import GemminiISA._
import Util._

class FgExecuteController[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit val p: Parameters, ev: Arithmetic[T]) 
  extends Module with HasCoreParameters {
  import config._
  import ev._
  //=========================================================================
  // I/O interface
  //=========================================================================
  val io = IO(new Bundle {
    val cmd        = Flipped(Decoupled(new GemminiCmd(rob_entries)))
    val readA      = new FgMemUnitExecReadIO(config, A_SP_FG_COLS)
    val readB      = new FgMemUnitExecReadIO(config, B_SP_FG_COLS)
    val writeC     = new FgMemUnitExecWriteReq(config)
    val acc_config = new FgAccumulatorBankConfigIO(config)
    val completed  = Valid(UInt(ROB_ENTRIES_IDX.W)) // TODO: make decoupled
    val busy       = Output(Bool())
    val prof       = Input(new Profiling)
  })

  io.cmd.ready        := DontCare
  io.readA.req        := DontCare
  io.readA.resp.data  := DontCare
  io.readB.req        := DontCare
  io.readB.resp.data  := DontCare
  io.writeC           := DontCare
  io.acc_config       := DontCare
  io.completed        := DontCare
  io.busy             := DontCare
}

object FgExecuteController {
  def apply[T <: Data: Arithmetic]
    (config: FgGemminiArrayConfig[T])(implicit p: Parameters)
      = Module(new FgExecuteController(config))
}
