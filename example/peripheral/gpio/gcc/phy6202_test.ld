/* PH6202 linker script */

/* Memory areas definitions */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x11000000, LENGTH = 512K

    ROM0 (rx)   : ORIGIN = 0x10000000, LENGTH = 8K
    ROM1 (rx)   : ORIGIN = 0x10002000, LENGTH = 120K
    
    RAM0 (rxw)  : ORIGIN = 0x1FFF0000, LENGTH = 32K
    RAM1 (rxw)  : ORIGIN = 0x1FFF8000, LENGTH = 32K
    RAM2 (rxw)  : ORIGIN = 0x20000000, LENGTH = 64K
    RAM3 (rxw)  : ORIGIN = 0x20010000, LENGTH = 8K
    RAM4 (rxw)  : ORIGIN = 0x20012000, LENGTH = 2K
}

/* Entry of the program: the reset handler */
/* Reset_Handler is defined in the starup file (assembly)*/
ENTRY(Reset_Handler)

/* Memory sections content */
SECTIONS
{
    .text :
    {
        KEEP(*(.vector*))
        *(.text*)
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4);
    } > FLASH

    .bss (NOLOAD) :
    {
        *(.bss*)
        *(COMMON)
    } > RAM0

    .data :
    {
        *(.data*);
    } > RAM1 AT > FLASH
}