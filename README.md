# USB - SPI bridge

it's a USB CDC device, supports transfers up to 64 bytes in length

## pinout 

 * PB12 - NSS/GPI0 (default)
 * PB13 - SCK
 * PB14 - MISO
 * PB15 - MOSI

 * PB3 - GPO1
 * PB4 - GPO2
 * PB5 - GPO3
 * PB6 - GPO4

 * PA0 - GPI0
 * PB1 - GPI1
 * PB2 - GPI2
 * PB3 - GPI3

 * A11 - DM
 * A12 - DP
 * A15 - USB PULLUP - resistor to A12


## Control endpoint messages:

Val==0x11 - select NSS pin
Index==0 -> GPO0
Index==1 -> GPO1
Index==2 -> GPO2
Index==3 -> GPO3
Index==4 -> GPO4

Val==0x12 - clear GPO
Val==0x13 - set GPO
Val==0x14 - read GPO
bit 0 - GPO0
bit 1 - GPO1
bit 2 - GPO2
bit 3 - GPO3
bit 4 - GPO4

Val==0x15 - read GPI
bit 0 - GPI0
bit 1 - GPI1
bit 2 - GPI2
bit 3 - GPI3


