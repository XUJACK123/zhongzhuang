ENTRY(Reset_Handler)

_Min_Heap_Size = 0x1000;
_Min_Stack_Size = 0x2000;

MEMORY
{
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  AXIRAM  (xrw) : ORIGIN = 0x24000000, LENGTH = 128K
  SRAM1   (xrw) : ORIGIN = 0x30000000, LENGTH = 16K
  SRAM2   (xrw) : ORIGIN = 0x30004000, LENGTH = 16K
  SRAM4   (xrw) : ORIGIN = 0x38000000, LENGTH = 16K
}

_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array (READONLY) :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >FLASH

  .init_array (READONLY) :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >FLASH

  .fini_array (READONLY) :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >DTCMRAM AT> FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >DTCMRAM

  .AXI_SRAM (NOLOAD) :
  {
    . = ALIGN(32);
    _saxisram = .;
    *(.AXI_SRAM)
    *(.AXI_SRAM*)
    . = ALIGN(32);
    _eaxisram = .;
  } >AXIRAM

  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCMRAM

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
