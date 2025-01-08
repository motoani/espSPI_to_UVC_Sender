// Transmits an image buffer via SPI as consecutive blocks
// Includes a basic checksum that can be used by the receiver
// Based on SPI master/slave example from ESP 
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "spi_sender.h"

// To synchronise with the receiver
// It initially started as a frame sync but now syncs each SPI block
EventGroupHandle_t frame_evnt_grp;

// The functions are not in the header as they aren't to be used outside this module

// This ISR is called when the handshake line goes low.
static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
{
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    static uint32_t lasthandshaketime_us;
    uint32_t currtime_us = esp_timer_get_time();
    uint32_t diff = currtime_us - lasthandshaketime_us;

    if (diff < 1000) {
        return; //ignore everything <1ms after an earlier irq
    }
    lasthandshaketime_us = currtime_us;

    //Give the semaphore.
    BaseType_t mustYield = false;
    // Example used a semaphore but I'm more comfortable woith event groups
    xEventGroupSetBitsFromISR(frame_evnt_grp, BLOCK_SYNC, &mustYield);
    if (mustYield) {
        portYIELD_FROM_ISR();
    }
} // End of gpio handshake

uint8_t make_checksum(const uint8_t * source, uint16_t source_len)
{
    // Calculate 8 bit checksum of an 8 bit source
    uint8_t chksum = 0;

    for (int i = 0; i < source_len; i++)
        {
            chksum += source[i];
        }
   return(chksum);
} // End of make_checksum 

// Variables local to this module
    
spi_device_handle_t handle;

uint8_t * block_buf_ptrs[TRANS_BLOCK_COUNT] = {NULL}; // Array of pointers to memory for each block

    //Configuration for the SPI bus
spi_bus_config_t send_buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    //Configuration for the SPI device on the other side of the bus
spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        // A faster clock will cause more errors and is limited by JPG compressor in receiver
        // ie 10 MHz can send at 30fps but Rx max at 24 fps
        // so best point is the lowest non-rate limiting clock
        .clock_speed_hz = 12000000, // 6Mhz should work
        .duty_cycle_pos = 128,      //50% duty cycle
        .mode = 1, //Default is 0 but see https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_slave.html#restrictions-and-known-issues
        .spics_io_num = GPIO_CS,
        .cs_ena_posttrans = 3,      //Keep the CS low at least 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size = TRANS_BLOCK_COUNT
    };

    //GPIO config for the handshake line.
gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pin_bit_mask = BIT64(GPIO_HANDSHAKE),
    };

    // Declare and clear the transaction queue items
spi_transaction_t spi_trans_queue[TRANS_BLOCK_COUNT];

void spi_send_init(void)
{
static char TAG[] = "SPI init";

        ESP_LOGI(TAG,"Setting up SPI sender");

        for (int qi = 0 ; qi < TRANS_BLOCK_COUNT; qi++)
        {
            memset(&spi_trans_queue[qi], 0, sizeof(spi_transaction_t));
        }
        
        // Allocate the block buffers and store their location
        // Buffers to hold a leading number, data and then byte checksum
        for (int qi = 0; qi < TRANS_BLOCK_COUNT; qi++ )
        {
            block_buf_ptrs[qi] = (uint8_t *)heap_caps_malloc(TRANS_BLOCK_SIZE + 2, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
            if (block_buf_ptrs[qi] == NULL)
            {
                ESP_LOGI(TAG,"Failed to allocate block buffer %d",(int)qi);
                exit(1);
            }
        }

        frame_evnt_grp = xEventGroupCreate();
        // Clear all event flags before starting tasks
        xEventGroupClearBits(frame_evnt_grp, BLOCK_SYNC);

        //Set up handshake line interrupt.
        ESP_ERROR_CHECK(gpio_config(&io_conf));   
        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        ESP_ERROR_CHECK(gpio_set_intr_type(GPIO_HANDSHAKE, GPIO_INTR_NEGEDGE));
        ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL));

        //Initialize the SPI bus and add the device we want to send stuff to.
        ESP_ERROR_CHECK(spi_bus_initialize(SENDER_HOST, &send_buscfg, SPI_DMA_CH_AUTO));
        ESP_ERROR_CHECK(spi_bus_add_device(SENDER_HOST, &devcfg, &handle));

        // Make a queue of transactions for all of the extended blocks
        for (int qi = 0; qi < TRANS_BLOCK_COUNT; qi++)
            {
                spi_trans_queue[qi].length = (TRANS_BLOCK_SIZE + 2) * 8;
                spi_trans_queue[qi].tx_buffer = block_buf_ptrs[qi]; //&imgbuf[qi * TRANS_BLOCK_SIZE / sizeof(uint16_t)];
                spi_trans_queue[qi].rx_buffer = NULL; // Shows nothing will be received in the full duplex mode
            }
} // End of spi_send_init 

// Currently only 128 * 128 is supported
// Although imgbuf is declared as uint8_t it is expected to be uint16_t RGB565
void spi_send(uint8_t * imgbuf)
{
    static char TAG[] = "SPI sender main";

        // NO handshake at this point as it's done per block
        //ESP_LOGI(TAG, "To send");

    // Do a check that init has happened
    if (block_buf_ptrs[0] == NULL)
    {
        ESP_LOGI(TAG, "SPI sender not initialised");
        exit (1);
    }
    // Copy the image buffer into the various block buffers
    // with index and checksums
    for (int qi = 0; qi < TRANS_BLOCK_COUNT; qi++)
        {
            // Include the block number
            * (block_buf_ptrs[qi] + 0) = (uint8_t) qi;
            // Copy the image data to the transmission block
            const uint8_t * src_ptr = (&imgbuf[qi * TRANS_BLOCK_SIZE]);
            memcpy((uint8_t *)(block_buf_ptrs[qi] + 2), src_ptr, TRANS_BLOCK_SIZE);
            // Second byte is the checksum
            * (block_buf_ptrs[qi] + 1) = make_checksum(src_ptr, TRANS_BLOCK_SIZE);
        }

       // Queue blocks for SPI transmission
        for (int qi = 0; qi < TRANS_BLOCK_COUNT; qi++)
        {
            // The approach with indexed transactions gives an option to 
            // queue as a batch although earlier tests weren't reliable
            // Require handshake for EVERY block but cycle through them all
            xEventGroupWaitBits(frame_evnt_grp, BLOCK_SYNC, true, true, portMAX_DELAY);

            // And queue the transaction now it's ready
            ESP_ERROR_CHECK(spi_device_queue_trans(handle, &spi_trans_queue[qi], portMAX_DELAY)); // Queue the checksum array transaction

            // Wait for this block to be sent
        {
            spi_transaction_t *rtrans;
            ESP_ERROR_CHECK(spi_device_get_trans_result(handle, &rtrans, portMAX_DELAY)); // Block until queue item finished
        }
            //ESP_LOGI(TAG,"Block %d sent",(int)qi);
        }

        //ESP_LOGI(TAG, "SPI sent");
 // End of sending a frame
 // No need to remove spi device as programme never quits
} // End of SPI sender
