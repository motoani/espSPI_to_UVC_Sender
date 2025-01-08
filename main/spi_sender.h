#pragma once

// Various parameters for the basic SPI image sender

// Defines how the image is broken down for transmission in SPI blocks
#define TRANS_BLOCK_SIZE 2048
#define TRANS_BLOCK_COUNT 16 // For a 128 x 128 px RGB565 image

#define SPI_WIDTH 128
#define SPI_HEIGHT 128

#define SENDER_HOST SPI2_HOST

// Use the default SPI GPIO lines, adapt handshake to suit
#define GPIO_HANDSHAKE      3
#define GPIO_MOSI           11
#define GPIO_MISO           -1 // No data from slave to master
#define GPIO_SCLK           12
#define GPIO_CS             10

#define BLOCK_SYNC 0x01 // Event group flag for handshake

    void spi_send_init(void);

    void spi_send(uint8_t * imgbuf);

