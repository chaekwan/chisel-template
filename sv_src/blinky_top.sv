// Generated by CIRCT firtool-1.44.0
module BlinkyCore (
    input  clock,
    reset,
    output io_led0
);

  reg       led;
  reg [8:0] counterWrap_c_value;
  always @(posedge clock or posedge reset) begin
    if (reset) begin
      led <= 1'h0;
      counterWrap_c_value <= 9'h0;
    end else begin
      automatic logic counterWrap;
      counterWrap = counterWrap_c_value == 9'h1F3;
      led <= counterWrap ^ led;
      if (counterWrap) counterWrap_c_value <= 9'h0;
      else counterWrap_c_value <= counterWrap_c_value + 9'h1;
    end
  end  // always @(posedge, posedge)
`ifdef ENABLE_INITIAL_REG_
`ifdef FIRRTL_BEFORE_INITIAL
  `FIRRTL_BEFORE_INITIAL
`endif  // FIRRTL_BEFORE_INITIAL
  initial begin
    if (reset) begin
      led = 1'h0;
      counterWrap_c_value = 9'h0;
    end
  end  // initial
`ifdef FIRRTL_AFTER_INITIAL
  `FIRRTL_AFTER_INITIAL
`endif  // FIRRTL_AFTER_INITIAL
`endif  // ENABLE_INITIAL_REG_
  assign io_led0 = led;
endmodule
