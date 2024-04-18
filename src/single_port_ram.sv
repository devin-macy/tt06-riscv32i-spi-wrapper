`ifndef H_RAM
`define H_RAM

`timescale 1ns / 1ns

// USED ONLY FOR TESTING THE SPI WRAPPER ALONE - NOT IN TOP LEVEL DESIGN

module single_port_ram #(
    parameter MEMFILE = "",        //! Name of file to initialize memory
    parameter MEM_DEPTH = 8,    //! Number of memory entries
    parameter DATA_WIDTH = 16     //! Word size
) (
    input logic                  clk,   //! Synchronous posedge clock
    input logic [$clog2(MEM_DEPTH):0] addr,  //! Memory word addressing
    input logic                  we,    //! Write enable
    input logic                  oe,    //! Output enable

    input logic [DATA_WIDTH-1:0] wr_data ,  //! Input/Output data line
    output logic [DATA_WIDTH-1:0] rd_data
);

  logic [DATA_WIDTH-1:0] tmp_data; //! Memory pointer
  logic [DATA_WIDTH-1:0] mem[MEM_DEPTH:0]; //! Memory bank

  initial begin
    if(MEMFILE != "")
      $readmemb(MEMFILE, mem);
  end
  
  always @(posedge clk) begin : write
    if (we) mem[addr] <= wr_data;
  end 

  always @(posedge clk) begin : read
    tmp_data <= mem[addr];
  end

  assign rd_data = (oe) ? tmp_data : rd_data;

endmodule

`endif