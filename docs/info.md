<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

My project is a riscv32i CPU core supporting most of the bare bones instruction set. The instructions that are not supported are any instructions dealing with CSRs, harts, memory fences, or modes of operation. The cpu core has 16 words (512 bytes) of instruction memory and registers and 8 words (256 bytes) of data memory.

For programmability, a SPI wrapper has been added that starts in boot mode, requiring you to upload a program and entering echo mode before the CPU can do anything. When the exit boot command is given to the SPI it will enter echo mode, releasing the cpu from reset, and repeat back any byte given to it as a sort of health check. In its current state, there is no way to observe the status of the CPU not in simulation since the SPI doesn't hand over control of itself to the CPU to output data.



## How to test

Explain how to use your project

## External hardware

List external hardware used in your project (e.g. PMOD, LED display, etc), if any
