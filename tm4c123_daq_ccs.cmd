/******************************************************************************
 *
 * tm4c123_daq_ccs.cmd - CCS linker configuration file for TM4C123G DAQ.
 *
 * TM4C123GH6PM: 256KB Flash, 32KB SRAM
 *
 *****************************************************************************/

--retain=g_pfnVectors

/* Application start address (interrupt vectors at 0x0000.0000) */
#define APP_BASE 0x00000000
#define RAM_BASE 0x20000000

/* System memory map */

MEMORY
{
    FLASH (RX) : origin = APP_BASE, length = 0x00040000
    SRAM (RWX) : origin = RAM_BASE, length = 0x00008000
}

/* Section allocation */

SECTIONS
{
    .intvecs:   > APP_BASE
    .text   :   > FLASH
    .const  :   > FLASH
    .cinit  :   > FLASH
    .pinit  :   > FLASH
    .init_array : > FLASH

    .vtable :   > RAM_BASE
    .data   :   > SRAM
    .bss    :   > SRAM
    .sysmem :   > SRAM
    .stack  :   > SRAM

#ifdef  __TI_COMPILER_VERSION__
#if     __TI_COMPILER_VERSION__ >= 15009000
    .TI.ramfunc : {} load=FLASH, run=SRAM, table(BINIT)
#endif
#endif
}

__STACK_TOP = __stack + 4096;
