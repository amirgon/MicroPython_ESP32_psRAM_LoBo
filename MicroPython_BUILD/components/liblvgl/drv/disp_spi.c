/**
 * @file disp_spi.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "disp_spi.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <string.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/


/**********************
 *  STATIC PROTOTYPES
 **********************/
static spi_device_handle_t spi;

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void disp_spi_init(void)
{

	esp_err_t ret;

	spi_bus_config_t buscfg={
		.miso_io_num=DISP_SPI_MISO,
		.mosi_io_num=DISP_SPI_MOSI,
		.sclk_io_num=DISP_SPI_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=128*1024,
	};

	spi_device_interface_config_t devcfg={
		.clock_speed_hz=DISP_SPI_MHZ*1000*1000, //Clock out at DISP_SPI_MHZ MHz
		.mode=0,                                //SPI mode 0
		.spics_io_num=DISP_SPI_CS,              //CS pin
		.queue_size=1,
		.pre_cb=NULL,
		.post_cb=NULL,
		.flags=SPI_DEVICE_HALFDUPLEX,
		.duty_cycle_pos=128,
	};

	gpio_pad_select_gpio(DISP_SPI_MISO);
    gpio_pad_select_gpio(DISP_SPI_MOSI);
    gpio_pad_select_gpio(DISP_SPI_CLK);

    gpio_set_direction(DISP_SPI_MISO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DISP_SPI_MISO, GPIO_PULLUP_ONLY);
    gpio_set_direction(DISP_SPI_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(DISP_SPI_CLK, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(DISP_SPI_CS);

	//Initialize the SPI bus
	ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	assert(ret==ESP_OK);

	//Attach the LCD to the SPI bus
	ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	assert(ret==ESP_OK);
}

void disp_spi_send(uint8_t * data, uint16_t length)
{
	if (length == 0) return;           //no need to send anything

	spi_transaction_t t;
    memset(&t, 0, sizeof(t));       	//Zero out the transaction
	t.length = length * 8;              //Length is in bytes, transaction length is in bits.
	t.tx_buffer = data;               	//Data

	esp_err_t ret;
//	ret=spi_device_transmit(spi, &t);  //Transmit!
//	assert(ret==ESP_OK);            	 //Should have had no issues.

	spi_device_queue_trans(spi, &t, portMAX_DELAY);

	spi_transaction_t * rt;
	spi_device_get_trans_result(spi, &rt, portMAX_DELAY);
}


/**********************
 *   STATIC FUNCTIONS
 **********************/
