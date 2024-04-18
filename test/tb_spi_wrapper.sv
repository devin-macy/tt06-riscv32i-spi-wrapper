`default_nettype none `timescale 1ns / 1ns


// command defenitions
parameter LOAD_LL = 8'hC0;
parameter LOAD_LH = 8'hC1;
parameter LOAD_HL = 8'hC2;
parameter LOAD_HH = 8'hC3;
parameter LOAD_ADDR = 8'hC4;
parameter WRITE_IMEM = 8'hC5;
parameter MODE_ECHO = 8'hC6;
parameter MODE_BOOT = 8'hC7;

module tb_spi_wrapper;

  // Parameters

  //Ports
  reg clk;
  reg rst_n;
  wire [7:0] rx_buff;
  wire rx_valid;
  reg [7:0] tx_buff;
  reg tx_valid;
  wire mode;
  wire cmd_error;
  wire cpu_rst_n;
  wire imem_wr_en;
  wire [31:0] prog_instr;
  wire [3:0] prog_addr;
  reg sclk;
  reg cs;
  reg mosi;
  wire miso;

  spi_wrapper DUT (
      .clk(clk),
      .rst_n(rst_n),
      .rx_buff(rx_buff),
      .rx_valid(rx_valid),
      .tx_buff(tx_buff),
      .tx_valid(tx_valid),
      .mode(mode),
      .cmd_error(cmd_error),
      .cpu_rst_n(cpu_rst_n),
      .imem_wr_en(imem_wr_en),
      .prog_instr(prog_instr),
      .prog_addr(prog_addr),
      .sclk(sclk),
      .cs(cs),
      .mosi(mosi),
      .miso(miso)
  );

  logic [31:0] data;
  localparam MEMFILE = "meminit.bin";
  localparam MEM_DEPTH = 16;
  localparam DATA_WIDTH = 32;
  single_port_ram # (
    .MEMFILE(MEMFILE),
    .MEM_DEPTH(MEM_DEPTH),
    .DATA_WIDTH(DATA_WIDTH)
  )
  imem (
    .clk(clk),
    .addr(prog_addr),
    .we(imem_wr_en),
    .oe(rst_n),
    .wr_data(prog_instr),
    .rd_data(data)
  );


  integer cycles = 0;
  initial clk = 1;
  always begin
    #5 clk = ~clk;
    cycles += 1;
  end

  initial begin
    $dumpfile("tb_spi_wrapper.vcd");
    $dumpvars(0, tb_spi_wrapper, imem);
    #1;
  end

  reg [127:0] command = {LOAD_LL, 8'haa,
                         LOAD_LH, 8'hbb,
                         LOAD_HL, 8'hcc,
                         LOAD_HH, 8'hdd,
                         LOAD_ADDR, 8'h09,
                         WRITE_IMEM, 8'h66,
                         MODE_ECHO, 8'h66, 
                         8'hCA, 8'hFE
  };

  // reg [31:0] command = {MODE_ECHO, 8'h66, 8'hA5, 8'h0F};

  assign mosi = command[$left(command)];

  always @(posedge sclk) begin
    command <= command << 1;
  end

  // print imem when write to memory
  always @(negedge imem_wr_en) begin
    print_imem();
  end

  always @(posedge sclk) begin
    if(!(DUT.tx_bit_count == 3'b111 && DUT.tx_bit_count_prev == 3'b111)) begin
      $write("%0b", miso);
    end
  end

  always_comb begin
    if(DUT.tx_bit_count == 3'b110 && DUT.tx_bit_count_prev == 3'b111) $display();
  end

  initial begin

    // reset
    rst_n <= 0;
    repeat (5) @(posedge clk);
    rst_n <= 1;

    // cycle chip select
    cs <= 1;
    repeat (5) @(posedge clk);
    cs <= 0;

    // receiving
    sclk <= 0;
    repeat (5) @(posedge clk);

    // BOOT + program
    spi_clk(12);
    repeat (50) @(posedge clk);
    
    // ECHO
    spi_clk(2);

    // tx and rx echo
    spi_clk(3);

    #100 sclk <= 1'b1;
    #100 sclk <= 1'b0;

    repeat (100) @(posedge clk);
    $write("\n");
    $finish();
  end

  task print_imem();
    $display("Instruction Memory (%0d)", cycles);
    for(integer i = 0; i < MEM_DEPTH; i++) begin
      $display("Address %3d = %0h", i, imem.mem[i]);
    end
  endtask

  task spi_clk(input [7:0] num_bytes);
    for (integer i = 0; i < num_bytes; i++) begin
      #200;
      for(integer j = 0; j < 16; j++) begin
        #100 sclk <= !sclk;
      end
    end
  endtask


endmodule
