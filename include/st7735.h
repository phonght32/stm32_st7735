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

#include "driver/gpio.h"
#include "driver/spi.h"

typedef struct st7735 *st7735_handle_t;

typedef enum {
	ST7735_COLOR_BLACK = 0,
	ST7735_COLOR_BLUE,
	ST7735_COLOR_RED,
	ST7735_COLOR_GREEN,
	ST7735_COLOR_CYAN,
	ST7735_COLOR_MAGENTA,
	ST7735_COLOR_YELLOW,
	ST7735_COLOR_WHITE,
	ST7735_COLOR_MAX
} st7735_color_t;

typedef enum {
	ST7735_SIZE_128_128 = 0,
	ST7735_SIZE_128_160,
	ST7735_SIZE_MAX
} st7735_size_t;

typedef enum {
	ST7735_ROT_0_DEG = 0,
	ST7735_ROT_90_DEG,
	ST7735_ROT_180_DEG,
	ST7735_ROT_270_DEG,
	ST7735_ROT_MAX,
} st7735_rot_t;

typedef struct {
	spi_num_t 			spi_num;
	spi_pins_pack_t 	spi_pins_pack;
	int 				gpio_port_rst;
	int 				gpio_num_rst;
	int 				gpio_port_cs;
	int 				gpio_num_cs;
	int 				gpio_port_dc;
	int 				gpio_num_dc;
	bool 				is_init;
} st7735_hw_info_t;

typedef struct {
	st7735_hw_info_t 	hw_info;
	st7735_size_t 		size;
	st7735_rot_t 		rot;
} st7735_cfg_t;

st7735_handle_t st7735_init(st7735_cfg_t *config);
stm_err_t st7735_fill(st7735_handle_t handle, uint16_t color);

#ifdef __cplusplus
}
#endif

#endif /* _ST7735_H_ */