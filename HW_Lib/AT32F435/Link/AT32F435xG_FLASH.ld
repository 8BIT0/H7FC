/*
*****************************************************************************
**
**  File        : AT32F435xG_FLASH.ld
**
**  Abstract    : Linker script for AT32F435xG Device with
**                1000KByte FLASH, 384KByte RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : Artery Tek AT32
**
**  Environment : Arm gcc toolchain
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
/* _estack = 0x20060000; */    /* end of RAM */

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x800;      /* required amount of heap  */
_Min_Stack_Size = 0x800; /* required amount of stack */

/* Specify the memory areas */

FLASH_SIZE = 1000K;
BOOT_SIZE = 256K;

MEMORY
{
FLASH (rx)       : ORIGIN = 0x08000000, LENGTH = FLASH_SIZE
FLASH_APP (rx)   : ORIGIN = ORIGIN(FLASH) + BOOT_SIZE, LENGTH = FLASH_SIZE - BOOT_SIZE
RAM_OsHeap (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
RAM_Perph (xrw)  : ORIGIN = 0x20020000, LENGTH = 64K
RAM (xrw)        : ORIGIN = 0x20030000, LENGTH = 192K
}

__rom_s = ORIGIN(FLASH);
__rom_e = __rom_s + FLASH_SIZE;
__boot_s = __rom_s;
__boot_e = __boot_s + BOOT_SIZE;

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH_APP

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    _stext = .;
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH_APP

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
    
    . = ALIGN(4);
    _shell_command_start = .;
    KEEP (*(shellCommand))
    _shell_command_end = .;
    . = ALIGN(4);
  } >FLASH_APP

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH_APP
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH_APP

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH_APP
  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH_APP
  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH_APP

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH_APP

  .OsHeap_Section :
  {
    . = ALIGN(4);
    OsHeap_s = .;
    *(.OsHeap_Section)
    *(.OsHeap_Section*)

    . = ALIGN(4);
    OsHeap_e = .;
  } >RAM_OsHeap AT>FLASH_APP

  .Perph_Section :
  {
    . = ALIGN(4);
    Perph_s = .;
    *(.Perph_Section)
    *(.Perph_Section*)
    
    . = ALIGN(4);
    Perph_e = .;
  } >RAM_Perph AT>FLASH_APP

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    _sstack = .;
    . = . + _Min_Stack_Size;
    _estack = .;
    . = ALIGN(8);
  } >RAM

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
