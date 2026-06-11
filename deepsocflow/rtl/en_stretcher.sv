module en_stretcher #(
  parameter W = 32,
  parameter N_CYCLES = 4   // how many cycles to stretch
)(
  input  logic clk,
  input  logic rstn,
  input  logic en_in,      // 1-cycle pulse
  output logic en_out      // stretched enable
);

  logic [$clog2(N_CYCLES):0] cnt;
  logic active;

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      cnt     <= '0;
      active  <= 1'b0;
      en_out  <= 1'b0;
    end else begin

      // Start stretching
      if (en_in && !active) begin
        active <= 1'b1;
        cnt    <= N_CYCLES - 1;
        en_out <= 1'b1;
      end

      // Continue stretching
      else if (active) begin
        if (cnt == 0) begin
          active <= 1'b0;
          en_out <= 1'b0;
        end else begin
          cnt    <= cnt - 1;
          en_out <= 1'b1;
        end
      end

      else begin
        en_out <= 1'b0;
      end
    end
  end

endmodule