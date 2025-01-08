# espSPI_to_UVC_Sender
The functions here are intended to be integrated into your graphical ESP32 application to interface with espSPI_to_UVC.
spi_sender acts as a SPI master and dicatates the clock frequency. Up to 12MHz seems to work. There is little advantage in going higher as other parts of the interface are rate-limiting. Note that the ESP32 hardware is not very resilient as a slave.

## Overview
A set of buffers are malloc'd by the initialisation function with space for a block of image plus a block identifier byte and a simple checksum byte. Whilst the block SPI transactions could be queued each transmission is triggered by the handshake line to ensure that the receiving slave is not overwhelmed. Nonetheless, it can take a few frames for the master and slave to be properly frame synchronised. Glitches may be visible on freeze-frames too! Ultimately, there are improvements which could be made...

The send function should be used to transmit your frame buffer after doing a TFT display. Ensure that GPIO pins are chosen to not conflict with any onboard TFT display. The Lilygo S3 T-display is ideal in this regard as it uses a I80 bus to interface with its display, leaving the user/peripheral SPI free for other uses.

## Code history
The function is based on the ESP SPI [slave example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/spi_slave/sender) but modified with a simple block and checksum protocol.