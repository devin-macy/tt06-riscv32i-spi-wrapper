<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

My project is a riscv32i cpu core supporting most of the bare bones instruction set. The instructions that are not supported are any instructions dealing with csr's, harts, memory fences, or modes of operation. The cpu core has 16 words (512 bytes) of instruction memory and registers and 8 words (256 bytes) of data memory.

For programmability, a spi wrapper has been added that starts in boot mode, requiring you to upload a program and entering echo mode before the cpu can do anything. When the exit boot command is given to the spi it will enter echo mode, releasing the cpu from reset, and repeat back any byte given to it as a sort of health check. 

In its current state, there is no way to observe the status of the cpu not in simulation, since the spi doesn't hand over control of itself to the cpu to output data. However the passed/failed signals check to see if register 10 is equal to 45, which you can upload a simple program that adds numbers from 1-10 and stores them into register 10 to test functionality.

## How to test

Have prepared a riscv32i binary of up to 16 instructions [here is a pretty good rescource for putting together binaries](https://riscvasm.lucasteske.dev/)

Connect the spi signals SCLK, CS, MOSI, and MISO to their respective pins. The spi commands are as follows:

|command|code|description|
|-------|----|-----------|
|load ll_byte       |0xc0     | load bits 7-0 of the instruction word                                                    |
|load lh_byte       |0xc1     | load bits 15-8 of the instruction word                                                   |
|load hl_byte       |0xc2     | load bits 23-16 of the instruction word                                                  |
|load hh_byte       |0xc3     | load bits 31-24 of the instruction word                                                  |
|load imem_addr     |0xc4     | load what 3-bit address you want to write the previously built instruction word to       |
|write to imem      |0xc5     | write built instruction to address loaded into imem_addr                                 |
|exit boot mode     |0xc6     | enter echo mode, echoing back any byte given afterwards                                  |
|re-enter boot mode |0xc7     | re-enter boot mode, holding cpu in reset                                                 |
|command error      |default  | invalid command was given, throw cmd_error on uo_out[5] high requiring a reset to clear  |

The spi commands require 2-bytes per command, even if the command doesnt use the second byte. 

## External hardware

male-to-male connectors and a arduino
