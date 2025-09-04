A purely passive I2C sniffer (and logger) using a simple RP2040 (Raspberry Pi Pico) utilizing its PIO block. This leaves the MCU to do other tasks while the PIO block deals with I2C.

In this case used to decode the I2C data going to a display to capture the data. 
Easy to adapt to do anything else with the I2C data.
