# stm32f411_usb
Minimal usb-from-scratch serial port implementation and demo for STM32F411. 

This is a rather bare-bones environment for the STM32F411.

It requires nothing but arm-none-eabi-gcc (tested with v9 and v10) and GNU Make to build and
has no dependencies on outside code.

Features:
- entirely plain C, no assembly
- customized header declares all devices as true structures, with their absolute addresses provided in the linker script.
- no #defines or conditional compilations.  All constants are enums. 
- compact printf implementation based on stb_printf, which can be used with any device for which you provide a puts() function.
- extremely small usb stack (1.7k) providing a serial port compatible with most operating systems

To re-use: just cut, copy and paste. Premature Generalisation is The Root Of Much Complexity. 

See stm32f103_usb for a simple userland client.  On Linux you can use the generic ttyUSBx driver.
