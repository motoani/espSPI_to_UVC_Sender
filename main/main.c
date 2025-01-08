
#include <stdint.h>
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h" 
#include "esp_dma_utils.h"

#include "esp_heap_caps_init.h"
#include "esp_timer.h"



#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include "driver/gpio.h"

#include "spi_sender.h"


// Random ranged number as per https://stackoverflow.com/questions/1202687/how-do-i-get-a-specific-range-of-numbers-from-rand
int my_random(int min, int max)
{
   return min + rand() / (RAND_MAX / (max - min + 1) + 1);
} // End of my_random

void app_main(void)
{
    char TAG[] = "main";

    // Make a local image buffer, need not be DMA capable
    // but should be byte addressable for SPI use
    uint16_t * this_image;
    this_image = (uint16_t *)heap_caps_malloc(SPI_WIDTH * SPI_HEIGHT * sizeof(uint16_t), MALLOC_CAP_8BIT);
    if (this_image == NULL)
    {
        ESP_LOGI(TAG,"Failed to allocate main image buffer");
        exit(1);
    }
    // Clear the image buffer
    for ( int x = 0; x < SPI_WIDTH; x ++)
    {
        for (int y = 0; y < SPI_HEIGHT; y++)
        {
            this_image[x + y * SPI_WIDTH] = 0x0000;
        }
    }
    // Set up the SPI sender, mostly malloc
    spi_send_init();

    // Send an image repeatedly to fill buffers
    // System is intended to be video, or at lease have ongoing transmissions
    ESP_LOGI(TAG, "SPI initialised so going to colour random pixels");
    while(1)
    {
        spi_send((uint8_t *)this_image);
        vTaskDelay(pdMS_TO_TICKS(20)); // Receiver will limit to 15fps in reality
        // Randomly colour pixels as RBG 565
        this_image[my_random(0,SPI_WIDTH-1) + my_random(0, SPI_HEIGHT-1) * SPI_WIDTH] = my_random(0, 0xffff); 
    }
}