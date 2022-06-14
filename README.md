# MIPS 32 bit processor SOC

Minimal SoC (System-on-a-Chip) containing a 32 bit MIPS processor, completely designed and implemented in VHDL. 

The computing system comprehend the core and the platform around it, mandatory for a working implementation on Zynq-7000 FPGA board.
The core, based on the open source Plasma CPU core, implements ALU, a radix-4 multiply/divide unit, control subsystem, memory controller, a three stage pipeline, 32 registers that are 32-bits wide, process counter and shifter.
The platform, developed for full compatibility with the Xilinx product, implements configurable separated data and instruction cache, bram block, an external memory controller and all the interconnects.
