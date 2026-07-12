MEMORY
{
    /* RAM-only layout used by the initial S31 loader port. Reserve enough
       executable SRAM for network stacks while retaining 192 KiB for data,
       heap and stacks. */
    /* espflash's S31 RAM stub occupies 0x2F001000..~0x2F001900 while
       downloading. Start the application above it so larger HAL images do
       not overwrite the running loader. */
    IRAM (RX)  : ORIGIN = 0x2F002000, LENGTH = 0x0002E000
    RAM  (RW)  : ORIGIN = 0x2F030000, LENGTH = 0x00030000
}
