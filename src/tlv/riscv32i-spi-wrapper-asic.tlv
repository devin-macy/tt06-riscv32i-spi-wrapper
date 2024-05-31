\m5_TLV_version 1d --inlineGen --noDirectiveComments --noline --clkAlways --bestsv --debugSigsYosys: tl-x.org
\m5
   use(m5-1.0)
   
   // #################################################################
   // #                                                               #
   // #  Starting-Point Code for MEST Course Tiny Tapeout RISC-V CPU  #
   // #                                                               #
   // #################################################################
   
   // ========
   // Settings
   // ========
   
   //-------------------------------------------------------
   // Build Target Configuration
   //
   // To build within Makerchip for the FPGA or ASIC:
   //   o Use first line of file: \m5_TLV_version 1d --inlineGen --noDirectiveComments --noline --clkAlways --bestsv --debugSigsYosys: tl-x.org
   //   o set(MAKERCHIP, 0)  // (below)
   //   o For ASIC, set my_design (below) to match the configuration of your repositoy:
   //       - tt_um_fpga_hdl_demo for tt_fpga_hdl_demo repo
   //       - tt_um_example for tt06_verilog_template repo
   //   o var(target, FPGA)  // or ASIC (below)
   set(MAKERCHIP, 0)   /// 1 for simulating in Makerchip.
   var(my_design, tt_um_riscv_spi_wrapper)   /// The name of your top-level TT module, to match your info.yml.
   var(target, ASIC)  /// FPGA or ASIC
   //-------------------------------------------------------
   
   // Input debouncing--not important for the CPU which has no inputs,but the setting is here for final projects based on the CPU.
   var(debounce_inputs, 0)         /// 1: Provide synchronization and debouncing on all input signals.
                                   /// 0: Don't provide synchronization and debouncing.
                                   /// m5_neq(m5_MAKERCHIP, 1): Debounce unless in Makerchip.

   // CPU configs
   var(num_regs, 32)  // 32 for full reg file.
   var(dmem_size, 32)  // Size of DMem, a power of 2.
   
   
   // ======================
   // Computed From Settings
   // ======================
   
   // If debouncing, a user's module is within a wrapper, so it has a different name.
   var(user_module_name, m5_if(m5_debounce_inputs, my_design, m5_my_design))
   var(debounce_cnt, m5_if_eq(m5_MAKERCHIP, 1, 8'h03, 8'hff))
   
\SV
   // Include Tiny Tapeout Lab.
   m4_include_lib(['https:/']['/raw.githubusercontent.com/os-fpga/Virtual-FPGA-Lab/35e36bd144fddd75495d4cbc01c4fc50ac5bde6f/tlv_lib/tiny_tapeout_lib.tlv'])  
   m4_include_lib(['https://raw.githubusercontent.com/efabless/chipcraft---mest-course/main/tlv_lib/risc-v_shell_lib.tlv'])


\TLV cpu()
   
   |cpu
      @0
         $reset = *reset || *spi_cpu_rst;
         
         $pc[31:0] = (>>1$reset)                     ? 32'd0 :
                     (>>3$valid_taken_br)            ? >>3$br_tgt_pc :  // if branch load pc with next instr after 2-cycle delay
                     (>>3$valid_load)                ? >>3$pc + 32'd4 : // if load load pc with next instr after 2-cycle delay
                     (>>3$valid_jump && >>3$is_jal)  ? >>3$br_tgt_pc :
                     (>>4$valid_jump && >>3$is_jalr) ? >>3$jalr_tgt_pc :
                                                       >>1$pc + 32'd4;
         
         // instruction fetch
         $imem_rd_en = !$reset;
         $imem_addr[3:0] = ($reset) ? *spi_prog_addr : $pc[5:2];
         
      @1 
         /imem[31:0]
            $wr = |cpu$imem_wr_en && (|cpu$imem_addr[4:0] == #imem);
            $value[31:0] = *reset ? 32'b0 :
                           $wr    ? |cpu$imem_wr_data :
                                    $RETAIN;
         ?$imem_rd_en
            $imem_rd_data[31:0] = /imem[$imem_addr[4:0]]>>1$value;
         
         
         $imem_wr_en = *spi_imem_wr_en;
         $imem_wr_data[31:0] = *spi_prog_instr;
         
         // decode
         $instr[31:0] = $imem_rd_data[31:0];
         
         // instruction types
         $is_i_instr = $instr[6:3] == 4'b0000 || $instr[6:2] == 5'b00100
                       || $instr[6:2] == 5'b00110 || $instr[6:2] == 5'b11001;
         $is_r_instr = $instr[6:2] == 5'b01011 || $instr[6:2] == 5'b01100
                       || $instr[6:2] == 5'b01110 || $instr[6:2] == 5'b10100;
         $is_u_instr = $instr[6:2] == 5'b00101 || $instr[6:2] == 5'b01101;
         $is_b_instr = $instr[6:2] == 5'b11000;
         $is_j_instr = $instr[6:2] == 5'b11011;
         $is_s_instr = $instr[6:3] == 4'b0100;
         $is_load = ($opcode == 7'b0000011);
         $is_jump = ($is_jal || $is_jalr);
         
         
         // immediate decode
         $imm[31:0] = ($is_i_instr) ? { {21{$instr[31]}}, $instr[30:20] } :
                      ($is_s_instr) ? { {21{$instr[31]}}, $instr[30:25], $instr[11:7] } :
                      ($is_b_instr) ? { {20{$instr[31]}}, $instr[7], $instr[30:25], $instr[11:8], 1'b0} :
                      ($is_u_instr) ? { $instr[31:12], 12'b0 } :
                      ($is_j_instr) ? { {12{$instr[31]}}, $instr[19:12], $instr[20], $instr[30:21], 1'b0} :
                                      // default
                                      32'b0;
         
         // validity of decoded fields
         $rd_valid = $is_r_instr || $is_i_instr || $is_u_instr || $is_j_instr;
         $rs1_valid = $is_r_instr || $is_i_instr || $is_s_instr || $is_b_instr;
         $rs2_valid = $is_r_instr || $is_s_instr || $is_b_instr;
         $func3_valid = $is_r_instr || $is_i_instr || $is_s_instr || $is_b_instr;
         $func7_valid = $is_r_instr;
         
         // operand and control decode
         $opcode[6:0] = $instr[6:0];
         ?$rd_valid
            $rd[4:0] = $instr[11:7];
         ?$rs1_valid
            $rs1[4:0] = $instr[19:15];
         ?$rs2_valid
            $rs2[4:0] = $instr[24:20];
         ?$func3_valid
            $func3[2:0] = $instr[14:12];
         ?$func7_valid
            $func7[6:0] = $instr[31:25];
         
         // instruction decoding (rv32i w/o R4 instrs)
         $dec_bits[10:0] = { $instr[30], $func3, $opcode };
         
         $is_lui = ($dec_bits[6:0] == 7'b0110111);
         $is_auipc = ($dec_bits[6:0] == 7'b0010111);
         $is_jal = ($dec_bits[6:0] == 7'b1101111);
         $is_jalr = ($dec_bits[6:0] == 7'b1100111);
         
         $is_beq = ($dec_bits[9:0] == 10'b000_1100011);
         $is_bne = ($dec_bits[9:0] == 10'b001_1100011);
         $is_blt = ($dec_bits[9:0] == 10'b100_1100011);
         $is_bge = ($dec_bits[9:0] == 10'b101_1100011);
         $is_bltu = ($dec_bits[9:0] == 10'b110_1100011);
         $is_bgeu = ($dec_bits[9:0] == 10'b111_1100011);
         
         //$is_lb = ($dec_bits[9:0] == 10'b000_0000011);
         //$is_lh = ($dec_bits[9:0] == 10'b001_0000011);
         //$is_lw = ($dec_bits[9:0] == 10'b010_0000011);
         //$is_lbu = ($dec_bits[9:0] == 10'b100_0000011);
         //$is_lhu = ($dec_bits[9:0] == 10'b101_0000011);
         
         //$is_sb = ($dec_bits[9:0] == 10'b000_0100011);
         //$is_sh = ($dec_bits[9:0] == 10'b001_0100011);
         //$is_sw = ($dec_bits[9:0] == 10'b010_0100011);
         
         $is_addi = ($dec_bits[9:0] == 10'b000_0010011);
         $is_slti = ($dec_bits[9:0] == 10'b010_0010011);
         $is_sltiu = ($dec_bits[9:0] == 10'b011_0010011);
         $is_xori = ($dec_bits[9:0] == 10'b100_0010011);
         $is_ori = ($dec_bits[9:0] == 10'b110_0010011);
         $is_andi = ($dec_bits[9:0] == 10'b111_0010011);
         
         $is_slli = ($dec_bits[10:0] == 11'b0_001_0010011);
         $is_srli = ($dec_bits[10:0] == 11'b0_101_0010011);
         $is_srai = ($dec_bits[10:0] == 11'b1_101_0010011);
         
         $is_add = ($dec_bits[10:0] == 11'b0_000_0110011);
         $is_sub = ($dec_bits[10:0] == 11'b1_000_0110011);
         $is_sll = ($dec_bits[10:0] == 11'b0_001_0110011);
         $is_slt = ($dec_bits[10:0] == 11'b0_010_0110011);
         $is_sltu = ($dec_bits[10:0] == 11'b0_011_0110011);
         $is_xor = ($dec_bits[10:0] == 11'b0_100_0110011);
         $is_srl = ($dec_bits[10:0] == 11'b0_101_0110011);
         $is_sra = ($dec_bits[10:0] == 11'b1_101_0110011);
         $is_or = ($dec_bits[10:0] == 11'b0_110_0110011);
         $is_and = ($dec_bits[10:0] == 11'b0_111_0110011);
         
      @2
         // register file (read)
         $rf_rd_en1 = $rs1_valid;
         $rf_rd_index1[4:0] = $rs1;
         $rf_rd_en2 = $rs2_valid;
         $rf_rd_index2[4:0] = $rs2;
         
         $src1_value[31:0] = (>>1$rf_wr_en && (>>1$rf_wr_index == $rf_rd_index1))
                             ? >>1$rf_wr_data : $rf_rd_data1;
         $src2_value[31:0] = (>>1$rf_wr_en && (>>1$rf_wr_index == $rf_rd_index2))
                             ? >>1$rf_wr_data : $rf_rd_data2;
         
         // branch target gen
         $br_tgt_pc[31:0] = $pc + $imm;
         $jalr_tgt_pc[31:0] = $src1_value + $imm;
         
      @3
         // register file (write)
         $rf_wr_en = ($rd_valid && $rd != 5'b0 && $valid) || >>2$valid_load;
         $rf_wr_index[4:0] = (>>2$valid_load) ? >>2$rd : $rd;
         $rf_wr_data[31:0] = (>>2$valid_load) ? >>2$ld_data : $result;
         
         // ALU
         /* verilator lint_off WIDTH */
         $sltu_rslt = $src1_value < $src2_value;
         $sltiu_rslt = $src1_value < $imm;
         
         $result[31:0] = ($is_andi)       ? $src1_value & $imm :
                         ($is_ori)        ? $src1_value | $imm :
                         ($is_xori)       ? $src1_value ^ $imm :
                         ($is_addi || $is_load || $is_s_instr)
                                          ? $src1_value + $imm :
                         ($is_slli)       ? $src1_value << $imm[5:0] :
                         ($is_srli)       ? $src1_value >> $imm[5:0] :
                         ($is_and)        ? $src1_value & $src2_value :
                         ($is_or)         ? $src1_value | $src2_value :
                         ($is_xor)        ? $src1_value ^ $src2_value :
                         ($is_add)        ? $src1_value + $src2_value :
                         ($is_sub)        ? $src1_value - $src2_value :
                         ($is_sll)        ? $src1_value << $src2_value[4:0] :
                         ($is_srl)        ? $src1_value >> $src2_value[4:0] :
                         ($is_sltu)       ? $sltu_rslt :
                         ($is_sltiu)      ? $sltiu_rslt :
                         ($is_lui)        ? {$imm[31:12], 12'b0} :
                         ($is_auipc)      ? $pc + $imm :
                         ($is_jal)        ? $pc + 32'd4 :
                         ($is_jalr)       ? $pc + 32'd4 :
                         ($is_srai)       ? { {32{$src1_value[31]} }, $src1_value} >> $imm[4:0] :
                         ($is_slt)        ? ($src1_value[31] == $src2_value[31]) ? $sltu_rslt : {31'b0, $src1_value[31]} :
                         ($is_slti)       ? ($src1_value[31] == $imm[31]) ? $sltiu_rslt : {31'b0, $src1_value[31]} :
                         ($is_sra)        ? { {32{$src1_value[31]} }, $src1_value} >> $src2_value[4:0] :
                                            32'bx;
         /* verilator lint_on WIDTH */
         
         // branch condition gen
         $taken_br = ($is_beq)  ? $src1_value == $src2_value :
                     ($is_bne)  ? $src1_value != $src2_value :
                     ($is_blt)  ? ($src1_value < $src2_value) ^ ($src1_value[31] != $src2_value[31]) :
                     ($is_bge)  ? ($src1_value >= $src2_value) ^ ($src1_value[31] != $src2_value[31]) :
                     ($is_bltu) ? $src1_value < $src2_value :
                     ($is_bgeu) ? $src1_value >= $src2_value :
                                  // default
                                  1'b0;
         
         // branch when valid
         $valid_taken_br = $valid && $taken_br;
         
         // load when valid
         $valid_load = $valid && $is_load;
         
         // jump when valid
         $valid_jump = $valid && $is_jump;
         
         // invanidate next 2-cycles for valid incorrect branch prediction, load instr, or jump
         $valid = ($reset) ? 1'b0 : !((>>1$valid_taken_br || >>2$valid_taken_br) || 
                                      (>>1$valid_load || >>2$valid_load) || 
                                      (>>1$valid_jump || >>2$valid_jump));
         
      @4
         // data memory
         $dmem_addr[2:0] = $result[4:2];
         $dmem_wr_en = $valid && $is_s_instr && (($result[4:2] != 3'b0) || ($result[4:2] != 3'b001) || ($result[4:2] != 3'b010));
         $dmem_wr_data[31:0] = $src2_value;
         $dmem_rd_en = $is_load && (($result[4:2] != 3'b0)|| ($result[4:2] != 3'b011));
         $ld_data[31:0] = ($result[4:2] == 3'b0)   ? $cycle_count :
                          ($result[4:2] == 3'b001) ? $spi_csr_out:
                          ($result[4:2] == 3'b010) ? $spi_current_instr :
                                                     $dmem_rd_data[31:0];
         
         // CSRs
         $cycle_count[31:0] = ($reset) ? 32'd0 : >>1$cycle_count + 1;  // dmem[0]
         $spi_csr_out[31:0] = *spi_csr[31:0];                          // dmem[1]
         $spi_current_instr[31:0] = *spi_prog_instr;                   // dmem[2]
         
      
      
      
   
   // Assert these to end simulation (before Makerchip cycle limit).
   // Note, for Makerchip simulation these are passed in uo_out to top-level module's passed/failed signals.
   *passed = |cpu/xreg[10]>>5$value == 45;
   *failed = ! *passed;
   
   // Connect Tiny Tapeout outputs. Note that uio_ outputs are not available in the Tiny-Tapeout-3-based FPGA boards.
   //*uo_out = {6'b0, *failed, *passed};
   //m5_if_neq(m5_target, FPGA, ['*uio_out = 8'b0;'])
   //m5_if_neq(m5_target, FPGA, ['*uio_oe = 8'b0;'])
   
   // Macro instantiations to be uncommented when instructed for:
   //  o instruction memory
   //  o register file
   //  o data memory
   //  o CPU visualization
   |cpu
      //m4+imem(@1)    // Args: (read stage)
      m4+rf(@2, @3)  // Args: (read stage, write stage) - if equal, no register bypass is required
      m4+dmem(@4)    // Args: (read/write stage)

   //m4+cpu_viz(@4)    // For visualisation, argument should be at least equal to the last stage of CPU logic. @4 would work for all labs.

\SV

// ================================================
// A simple Makerchip Verilog test bench driving random stimulus.
// Modify the module contents to your needs.
// ================================================

module top(input logic clk, input logic reset, input logic [31:0] cyc_cnt, output logic passed, output logic failed);
   // Tiny tapeout I/O signals.
   logic [7:0] ui_in, uo_out;
   m5_if_neq(m5_target, FPGA, ['logic [7:0] uio_in,  uio_out, uio_oe;'])
   m5_if_neq(m5_target, FPGA, ['assign uio_in = 8'b0;'])
   logic ena = 1'b0;
   logic rst_n = ! reset;
   
   // Instantiate the Tiny Tapeout module.
   m5_user_module_name tt(.*);
   
   // Passed/failed to control Makerchip simulation, passed from Tiny Tapeout module's uo_out pins.
   //assign passed = uo_out[6];
   //assign failed = uo_out[7];
endmodule


// Provide a wrapper module to debounce input signals if requested.
m5_if(m5_debounce_inputs, ['m5_tt_top(m5_my_design)'])
\SV

\SV                
module spi_wrapper
(
  // Control/Data Signals
  input logic clk,                       // system clock
  input logic rst_n,                     // active-low reset
  
  // CSR
  output logic [7:0] rx_buff,             // MOSI buffer - populates when SPI recieves a full byte
  output logic rx_valid,                  // pulsed if successfully recieved a full byte
  input logic [7:0] tx_buff,             // MISO buffer - pulled into r_tx_buff when tx_valid is set high by the CPU (NOT IMPLEMENTED)
  input logic tx_valid,                   // pulsed if ready to transmit data (NOT IMPLEMENTED)
  output logic mode,                      // 0 if boot, 1 if echo
  output logic cmd_error,                 // asserts when an invalid cmd is given, must reset to clear
  
  // CPU program signals
  output logic cpu_rst_n,                // hold CPU in reset when programming in boot mode
  output logic imem_wr_en,                // write enable for instruction memory
  output logic [31:0] prog_instr,         // instruction used to write to memory
  output logic [3:0] prog_addr,           // address used to write to memory
  
  // SPI Interface
  input logic sclk,                      // SPI clock
  input logic cs,                        // chip select (active-low)
  input logic mosi,                      // SPI recieve data
  output logic miso                      // SPI transmit data
);
  
  logic [2:0] rx_bit_count;           
  logic [7:0] r_rx_buff;              
  logic [7:0] r_rx_buff_temp;         
  logic rx1_done, rx2_done, rx3_done;    // clock domain crossing signals
   
  logic [7:0] rx_cmd;            		   // command recieved in the last byte
  logic rx_grab_cmd_n;           		   // flip-flop between decoding command
                                       // or operating on current data byte
 
  logic [2:0] tx_bit_count;
  logic [2:0] tx_bit_count_prev;             // used to invalidate echo response
  logic [7:0] r_tx_buff;
  logic response_valid;                  // echo mode internal tx start signal
  logic miso_bit;                         
  assign miso = miso_bit;
  
  logic [7:0] hh_byte;                   // instruction [31:24]
  logic [7:0] hl_byte;                   // instruction [23:16]
  logic [7:0] lh_byte;                   // instruction [15:8]
  logic [7:0] ll_byte;                   // instruction [7:0]
  logic [3:0] imem_address;              
  
   
  // rx spi and global cock domain crossing
  // assert rx_valid for 1 cycle when a full byte is recieved
  always @(posedge clk) begin
     if (~rst_n) begin // sync reset
        rx2_done <= 1'b0;
        rx3_done <= 1'b0;
        
        rx_valid <= 1'b0;
        rx_buff <= 8'b00;
     end else begin
        // clock domain crossing between clk and sclk
     	  rx2_done <= rx1_done;
        rx3_done <= rx2_done;
        
        if(rx3_done == 1'b0 && rx2_done == 1'b1) begin // rising edge
           // done recieving, set rx data valid and load output buffer
           rx_valid <= 1'b1;
           rx_buff <= r_rx_buff;
        end else begin
           // clear rx data valid, rx_buff persists
           rx_valid <= 1'b0;
        end
     end
  end
  // receive mosi bits from spi clk
  always @(posedge sclk, posedge cs) begin
	  if (cs) begin // hold in reset when not selected
        rx_bit_count <= 3'b0;
        rx1_done <= 1'b0;
     end else begin
        rx_bit_count <= rx_bit_count + 1;              // increment bit count
        r_rx_buff_temp <= {r_rx_buff_temp[6:0], mosi}; // shift in bits MSB first to temp buffer
        
        if (rx_bit_count == 3'b111) begin
           rx1_done <= 1'b1;                           // signal recieved full byte 
           r_rx_buff <= {r_rx_buff_temp[6:0], mosi};   // shift in last bit to temp buffer
        end else if(rx_bit_count == 3'b010) begin      // de-assert when starting to recieve next byte
           rx1_done <= 1'b0;
        end
     end
  end
   
  // register tx byte when tx_valid pulse comes, initiating transfer
  // NOT IMPLEMENTED - driving signals tied to ground and caused multiple driver errors on r_tx_buff
  /*
  always @(posedge clk) begin
     if(~rst_n) begin
        r_tx_buff <= 8'h00;
     end else begin
        if(tx_valid) begin
           r_tx_buff <= tx_buff;
        end
     end
  end
  */
  
  // clock out tx byte when there is a tx byte (echo)
  always @(posedge sclk, posedge cs) begin
     if (cs) begin
        tx_bit_count <= 3'b111;   // send MSB first
        tx_bit_count_prev <= 3'b111;
        miso_bit <= r_tx_buff[7]; // reset to MSB
     end else begin
        // with 2 modes - only able to tx if in echo mode
        // future work would implement cpu control over tx_buff and tx_valid
        if (mode == 1'b1) begin
           tx_bit_count_prev <= tx_bit_count;
           tx_bit_count <= tx_bit_count - 1;
           miso_bit <= r_tx_buff[tx_bit_count];
        end else begin
           tx_bit_count <= 3'b111;   
        end
     end
  end
  
  // internal spi commands
  always @(posedge clk) begin
     if (~rst_n) begin // sync reset
        
        cpu_rst_n <= 1'b0;
        mode <= 1'b0;
        cmd_error <= 1'b0;
        
        hh_byte <= 8'h00;
        hl_byte <= 8'h00;
        lh_byte <= 8'h00;
        ll_byte <= 8'h00;
        prog_instr <= 32'h00_00_00_00;
        
        imem_wr_en <= 1'b0;
        imem_address <= 4'h0;
        prog_addr <= 4'h0;
        
        rx_grab_cmd_n <= 1'b0;
        rx_cmd <= 8'h00;
        
        response_valid <= 1'b0;
        r_tx_buff <= 8'h00;
     end else if (rx3_done == 1'b0 && rx2_done == 1'b1) begin // if recieved a full byte - rising-edge
        if (~mode) begin // boot mode
           cpu_rst_n <= 1'b0;                                 // assert cpu reset when in boot mode
           if (~rx_grab_cmd_n) begin // command
              rx_cmd <= r_rx_buff;
              rx_grab_cmd_n <= 1'b1;                          // next byte is data
           end else if(rx_grab_cmd_n) begin // data
              case (rx_cmd)                                   // decode last command and operate on current byte
                8'hc0 : begin
                         ll_byte <= r_rx_buff;
                        end
                8'hc1 : begin 
                         lh_byte <= r_rx_buff; 
                        end
                8'hc2 : begin 
                         hl_byte <= r_rx_buff; 
                        end
                8'hc3 : begin 
                         hh_byte <= r_rx_buff; 
                        end
                8'hc4 : begin 
                         imem_address <= r_rx_buff[3:0];
                        end
                8'hc5 : begin
                         prog_addr <= imem_address;
                         prog_instr <= {hh_byte, hl_byte, lh_byte, ll_byte};
                         imem_wr_en <= 1'b1;
                        end
                8'hc6 : begin 
                         mode <= 1'b1;                        // enter echo mode
                         cpu_rst_n <= 1'b1;                   // de-assert cpu reset when not in boot
                        end
                8'hc7 : begin
                         mode <= 1'b0;                        // do nothing
                         end
                default: begin 
                         cmd_error <= 1'b1;                   // invalid cmd, must reset to clear
                         end
               endcase
               rx_grab_cmd_n <= 1'b0;                         // next byte is a cmd
           end // end rx_grab_cmd_n
        end else begin // echo mode
         if(r_rx_buff == 8'hc7) begin                       // re-enter boot if in echo mode and cmd is recieved
            mode <= 1'b0;
         end
         response_valid <= 1'b1;
         r_tx_buff <= r_rx_buff;                            // load TRANSMIT buffer with RECIEVED buffer 
        end // end mode
     end else if(tx_bit_count_prev == 3'b000) begin                  // de-assert response_valid after tx complete
        response_valid <= 1'b0;                               // (can probably move up into echo mode case)
     end else begin
        imem_wr_en <= 1'b0;                                   // de-assert write enable on cmd (only happens when doing nothing in boot mode)
     end // end byte recieved
  end // end spi internal command
endmodule

                   
\SV



// =======================
// The Tiny Tapeout module
// =======================

module m5_user_module_name (
    input  wire [7:0] ui_in,    // Dedicated inputs - connected to the input switches
    output wire [7:0] uo_out,   // Dedicated outputs - connected to the 7 segment display
    //m5_if_eq(m5_target, FPGA, ['/']['*'])   // The FPGA is based on TinyTapeout 3 which has no bidirectional I/Os (vs. TT6 for the ASIC).
    input  wire [7:0] uio_in,   // IOs: Bidirectional Input path
    output wire [7:0] uio_out,  // IOs: Bidirectional Output path
    output wire [7:0] uio_oe,   // IOs: Bidirectional Enable path (active high: 0=input, 1=output)
    //m5_if_eq(m5_target, FPGA, ['*']['/'])
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);
   logic passed, failed; 
   assign uo_out[7:6] = {failed, passed};
   
   // [31:18] - unused
   // [17] [16:9] - tx_valid + tx_buff (NOT IMPLEMENTED)
   // [8] [7:0] - rx_valid + rx_buff
   logic [31:0] spi_csr;
   logic [31:0] spi_prog_instr;
   logic [3:0] spi_prog_addr;
   logic spi_imem_wr_en;
   logic spi_cpu_rst_n;
   
   // resets
   logic spi_cpu_rst = ! spi_cpu_rst_n;
   wire reset = ! rst_n;
   
   // unused signals
   assign uio_oe = 8'b0;
   assign uio_out = 8'b0;
   assign uo_out[2:0] = 3'b0;
   assign spi_csr[31:18] = 14'b0;
   assign spi_csr[17:9] = 9'b0;
   
   spi_wrapper spi (
		.clk(clk),
      .rst_n(rst_n),
      
      // control and status registers
      .mode(uo_out[4]),
      .rx_buff(spi_csr[7:0]),
      .rx_valid(spi_csr[8]),
      .tx_buff(spi_csr[16:9]),
      .tx_valid(spi_csr[17]),
      
      // cpu boot signals
      .cpu_rst_n(spi_cpu_rst_n),
      .cmd_error(uo_out[5]),
      .imem_wr_en(spi_imem_wr_en),
      .prog_instr(spi_prog_instr),
      .prog_addr(spi_prog_addr),
      
      // spi interface
      .sclk(ui_in[0]),
      .cs(ui_in[1]),
      .mosi(ui_in[2]),
      .miso(uo_out[3])
      
   );
   
\TLV
   /* verilator lint_off UNOPTFLAT */
   // Connect Tiny Tapeout I/Os to Virtual FPGA Lab.
   m5+tt_connections()
   
   // Instantiate the Virtual FPGA Lab.
   m5+board(/top, /fpga, 7, $, , cpu)
   // Label the switch inputs [0..7] (1..8 on the physical switch panel) (top-to-bottom).
   m5+tt_input_labels_viz(['"UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED"'])

\SV
endmodule
