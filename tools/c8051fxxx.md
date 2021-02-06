# C8051Fxxx

## Basics

* MCS-51™ is a instruction set run on 8051.

## Flash Memory

* CPU is stalled during flash write / erase operations, but peripherals remain active. Interrupts are posted until completeness of flash operations and are serviced in priority order.
* \(F340 specific\)Program memory consists of 64kiB of flash\(0x0000~0xFFFF\), organized into a set of 512B pages. The space from 0xFC00 to 0xFFFF is **reserved**.
* \(F34x specific\)A Flash Security Lock Byte located at the last byte of Flash user space offers protection of program memory. Assume that the value of Lock Byte is `x`, then `~x` successive pages starting at page 0x0000~0x01FF are locked. If there are other Flash pages locked, the page containing the Lock Byte is also locked.
  * 设计为取反或许是因为 flash erase 时置 1 的特性
  * \(F340 specific\) The page containing the Lock Byte is 0xFA00~0xFBFF\(that is, the last page of user space\).

![](../.gitbook/assets/image-20210131113200394.png)

