/* STM32H743VIT6 Memory Layout */
/* Running with Aviate bootloader at 0x08020000 */

MEMORY
{
    /* Flash: App region starts after 128KB bootloader */
    FLASH : ORIGIN = 0x08020000, LENGTH = 1920K

    /* RAM regions */
    DTCMRAM : ORIGIN = 0x20000000, LENGTH = 128K
    RAM_D1  : ORIGIN = 0x24000000, LENGTH = 512K
    RAM_D2  : ORIGIN = 0x30000000, LENGTH = 288K
    RAM_D3  : ORIGIN = 0x38000000, LENGTH = 64K
    ITCMRAM : ORIGIN = 0x00000000, LENGTH = 64K
}

/* Use D1 RAM as main RAM */
REGION_ALIAS("RAM", RAM_D1);

/* Stack */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
