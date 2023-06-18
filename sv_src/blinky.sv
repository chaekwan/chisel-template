// Generated by CIRCT firtool-1.44.0
module Blinky(
  input  clock,
         reset,
  output io_led0
);

  reg       led;
  reg [8:0] counterWrap_c_value;
  always @(posedge clock) begin
    if (reset) begin
      led <= 1'h0;
      counterWrap_c_value <= 9'h0;
    end
    else begin
      automatic logic counterWrap = counterWrap_c_value == 9'h1F3;
      led <= counterWrap ^ led;
      if (counterWrap)
        counterWrap_c_value <= 9'h0;
      else
        counterWrap_c_value <= counterWrap_c_value + 9'h1;
    end
  end // always @(posedge)
  assign io_led0 = led;
endmodule
