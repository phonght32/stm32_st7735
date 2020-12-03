// MIT License

// Copyright (c) 2020 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef _ST7735_H_
#define _ST7735_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stm_err.h"

#include "fonts.h"
#include "driver/gpio.h"
#include "driver/spi.h"

typedef struct st7735 *st7735_handle_t;

typedef enum {
	ST7735_SIZE_128_128 = 0,					/*!< Screen resolution 128x128 */
	ST7735_SIZE_128_160,						/*!< screen resolution 128x160 */
	ST7735_SIZE_MAX
} st7735_size_t;

typedef enum {
	ST7735_ROT_0_DEG = 0,						/*!< Screen rotate 0 degree */
	ST7735_ROT_90_DEG,							/*!< Screen rotate 90 degree */
	ST7735_ROT_180_DEG,							/*!< Screen rotate 180 degree */
	ST7735_ROT_270_DEG,							/*!< Screen rotate 270 degree */
	ST7735_ROT_MAX,
} st7735_rot_t;

typedef struct {
	spi_num_t 			spi_num;				/*!< SPI num */
	spi_pins_pack_t 	spi_pins_pack;			/*!< SPI pins pack */
	int 				gpio_port_rst;			/*!< GPIO port reset */
	int 				gpio_num_rst;			/*!< GPIO num reset */
	int 				gpio_port_cs;			/*!< GPIO port chip select */
	int 				gpio_num_cs;			/*!< GPIO num chip select */
	int 				gpio_port_dc;			/*!< GPIO port data/command */
	int 				gpio_num_dc;			/*!< GPIO num data/command */
	bool 				is_init;				/*!< Check if hardware is initialized */
} st7735_hw_info_t;

typedef struct {
	st7735_hw_info_t 	hw_info;				/*!< Hardware information */
	st7735_size_t 		size;					/*!< Screen resolution */
	st7735_rot_t 		rot;					/*!< Screen rotation */
} st7735_cfg_t;

/*
 * @brief   Initialize ST7735 driver.
 * @param   config Struct pointer.
 * @return
 *      - ST7735 handle structure: Success.
 *      - 0: Fail.
 */
st7735_handle_t st7735_init(st7735_cfg_t *config);

/*
 * @brief   Fill screen with color.
 * @param   handle Handle structure.
 * @param 	color Fill color.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t st7735_fill_screen(st7735_handle_t handle, uint16_t color);

/*
 * @brief   Write ASCII character.
 * @param   handle Handle structure.
 * @param 	font_size Font size.
 * @param 	chr ASCII character.
 * @param 	color Color.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t st7735_write_char(st7735_handle_t handle, font_size_t font_size, uint8_t chr, uint16_t color);

/*
 * @brief   Write string.
 * @param   handle Handle structure.
 * @param 	font_size Font size.
 * @param 	str Pointer to strings.
 * @param 	color Color.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t st7735_write_string(st7735_handle_t handle, font_size_t font_size, uint8_t *str, uint16_t color);

/*
 * @brief   Draw color to pixel.
 * @param   handle Handle structure.
 * @param 	x Column index.
 * @param 	y Row index.
 * @param 	color Color.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t st7735_draw_pixel(st7735_handle_t handle, uint8_t x, uint8_t y, uint16_t color);

/*
 * @brief   Draw line.
 * @param   handle Handle structure.
 * @param 	x_start x start position.
 * @param 	y_start y start position.
 * @param 	x_end x end position.
 * @param 	y_end y end position.
 * @param 	color Line color.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t st7735_draw_line(st7735_handle_t handle, uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end, uint16_t color);

/*
 * @brief   Draw rectangle.
 * @param   handle Handle structure.
 * @param 	x_origin x origin position.
 * @param 	y_origin y origin position.
 * @param 	width Width.
 * @param 	height Height.
 * @param 	color Line color.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t st7735_draw_rectangle(st7735_handle_t handle, uint8_t x_origin, uint8_t y_origin, uint8_t width, uint8_t height, uint16_t color);

/*
 * @brief   Draw circle.
 * @param   handle Handle structure.
 * @param 	x_origin x origin position.
 * @param 	y_origin y origin position.
 * @param 	radius Radius.
 * @param 	color Line color.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t st7735_draw_circle(st7735_handle_t handle, uint8_t x_origin, uint8_t y_origin, uint8_t radius, uint16_t color);

/*
 * @brief   Draw mono image.
 * @param   handle Handle structure.
 * @param 	x_origin x origin position.
 * @param 	y_origin y origin position.
 * @param 	width Image width in pixel.
 * @param 	height Image height in pixel.
 * @param 	image_src Image source.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t st7735_draw_image(st7735_handle_t handle, uint8_t x_origin, uint8_t y_origin, uint8_t width, uint8_t height, uint8_t *image_src);

/*
 * @brief   Set position.
 * @param   handle Handle structure.
 * @param 	x Column index.
 * @param 	y Row index.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t st7735_set_position(st7735_handle_t handle, uint8_t x, uint8_t y);

/*
 * @brief   Get position.
 * @param   handle Handle structure.
 * @param 	x Pointer to column index.
 * @param 	y Pointer to row index.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t st7735_get_position(st7735_handle_t handle, uint8_t *x, uint8_t *y);

/*
 * @brief   Destroy ST7735 handle structure.
 * @param   handle Handle structure.
 * @return	None.
 */
void st7735_destroy(st7735_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* _ST7735_H_ */