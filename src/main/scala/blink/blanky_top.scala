import chisel3._
import chisel3.util.Counter
import circt.stage.ChiselStage
import chisel3.{RawModule, withClockAndReset}

class BlinkyCore(freq: Int, startOn: Boolean = false) extends Module with RequireAsyncReset {
  val io = IO(new Bundle {
    val led0 = Output(Bool())
  })
  // Blink LED every second using Chisel built-in util.Counter
  val led = RegInit(startOn.B)
  val (_, counterWrap) = Counter(true.B, freq / 2)
  when(counterWrap) {
    led := ~led
  }
  io.led0 := led
}

// class BlinkyTop extends RawModule {
//   val io = IO(new Bundle {
//   val led0 = Output(Bool())
//   val clk = IO(Input(Clock()))
//   val rst_n = IO(Input(AsyncReset()))
//   })

// val blinky = withClockAndReset(io.clk, io.rst_n){ Module(new BlinkyCore(1000)) }

//   io.led0 := blinky.io.led0
// }

object BlinkyTopMain extends App {
  // These lines generate the Verilog output
  println(
    ChiselStage.emitSystemVerilog(
      new BlinkyTop(),
      firtoolOpts =
        Array("-disable-all-randomization", "-strip-debug-info", "-o", "sv_src/blinky_top.sv")
    )
  )
}