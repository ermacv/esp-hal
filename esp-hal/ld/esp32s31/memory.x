MEMORY
{
    /* Conservative RAM-only layout used by the initial S31 loader port.
       The low 64 KiB window holds executable code; the following 448 KiB
       holds data, heap and stacks. */
    /* espflash's S31 RAM stub occupies 0x2F001000..~0x2F001900 while
       downloading. Start the application above it so larger HAL images do
       not overwrite the running loader. */
    IRAM (RX)  : ORIGIN = 0x2F002000, LENGTH = 0x0000E000
    RAM  (RW)  : ORIGIN = 0x2F010000, LENGTH = 0x00050000
}
