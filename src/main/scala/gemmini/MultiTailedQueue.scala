package gemmini

import chisel3._
import chisel3.util._
import Util._

class MultiTailedQueue[T <: Data](gen: T, entries: Int, 
                                  tails: Int, maxpush: Int = 2) extends Module 
{
  val io = IO(new Bundle {
    val enq = new Bundle {
      val ready = Output(Vec(tails, Bool()))
      val bits = Input(Vec(tails, gen))
      val push = Input(UInt(log2Ceil((entries min maxpush) + 1).W))
    }
    val deq = Flipped(Decoupled(gen))
    val len = Output(UInt(log2Ceil(entries+1).W))
  })

  val regs  = Reg(Vec(entries, gen))
  val raddr = RegInit(0.U((log2Ceil(entries) max 1).W))
  val waddr = RegInit(0.U((log2Ceil(entries) max 1).W))
  val len   = RegInit(0.U(log2Ceil(entries+1).W))

  assert(tails >= 1)
  assert(io.enq.push <= len)
  assert(io.enq.push <= tails.U)
  assert(io.enq.push <= maxpush.U)

  // push interface
  for (i <- 0 until tails) {
    io.enq.ready(i) := len < (entries - i).U
  }
  waddr := wrappingAdd(waddr, io.enq.push, entries)

  // pop interface
  io.deq.bits := regs(raddr)
  io.deq.valid := len > 0.U
  raddr := wrappingAdd(raddr, io.deq.fire(), entries)

  // length calc
  io.len := len
  len := len + io.enq.push - io.deq.fire()
}

object MultiTailedQueue {
  def apply[T <: Data](dst: ReadyValidIO[T], entries: Int, tails: Int) = {
    val q = Module(new MultiTailedQueue(dst.bits.cloneType, entries, 
                                        tails: Int))
    q.io.deq <> dst
    (q.io.enq, q.io.len)
  }
}
