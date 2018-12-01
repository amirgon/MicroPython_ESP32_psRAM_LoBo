/**
 * @file lv_templ.h
 *
 */

#ifndef LV_TEMPL_H
#define LV_TEMPL_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdint.h>

/*********************
 *      DEFINES
 *********************/

#define DISP_SPI_MHZ 8
#define DISP_SPI_MISO 5
#define DISP_SPI_MOSI 18
#define DISP_SPI_CLK  19
#define DISP_SPI_CS   13





/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void disp_spi_init(void);
void disp_spi_send(uint8_t * data, uint16_t length);

/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_TEMPL_H*/
