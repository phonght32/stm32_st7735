#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "include/st7735.h"

#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MV  0x20
#define ST7735_MADCTL_ML  0x10
#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH  0x04

#define ST7735_XSTART 2
#define ST7735_YSTART 3
#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR)

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_SET_COLUMN_ADDR   0x2A
#define ST7735_SET_ROW_ADDR   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Color definitions
#define	ST7735_BLACK   0x0000
#define	ST7735_BLUE    0x001F
#define	ST7735_RED     0xF800
#define	ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF
#define ST7735_COLOR565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

#define TIMEOUT_MS				200

#define ST7735_INIT_ERR_STR				"st7735 init error"
#define ST7735_WRITE_CMD_ERR_STR		"st7735 write command error"
#define ST7735_WRITE_DATA_ERR_STR		"st7735 write data error"
#define ST7735_FILL_ERR_STR				"st7735 file error"

#define mutex_lock(x)			while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x) 		xSemaphoreGive(x)
#define mutex_create()			xSemaphoreCreateMutex()
#define mutex_destroy(x) 		vQueueDelete(x)

static const char *TAG = "ST7735";
#define ST7735_CHECK(a, str, action) if(!(a)) {								\
	STM_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);	\
	action;																	\
}

// Init for 7735R, part 1 (red or green tab)
static uint8_t init_cmds1[] = {
	15,                       // 15 commands in list:
	ST7735_SWRESET,   0x80,  //  1: Software reset, 0 args, w/delay
	150,                    //     150 ms delay
	ST7735_SLPOUT ,   0x80,  //  2: Out of sleep mode, 0 args, w/delay
	255,                    //     500 ms delay
	ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
	0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
	ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
	0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
	ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
	0x01, 0x2C, 0x2D,       //     Dot inversion mode
	0x01, 0x2C, 0x2D,       //     Line inversion mode
	ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
	0x07,                   //     No inversion
	ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
	0xA2,
	0x02,                   //     -4.6V
	0x84,                   //     AUTO mode
	ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
	0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
	ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
	0x0A,                   //     Opamp current small
	0x00,                   //     Boost frequency
	ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
	0x8A,                   //     BCLK/2, Opamp current small & Medium low
	0x2A,
	ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
	0x8A, 0xEE,
	ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
	0x0E,
	ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
	ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
	ST7735_ROTATION,        //     row addr/col addr, bottom to top refresh
	ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
	0x05					//     16-bit color
};

uint8_t init_cmds2[] = {            // Init for 7735R, part 2 (1.44" display)
	2,                        //  2 commands in list:
	ST7735_SET_COLUMN_ADDR  , 4      ,  //  1: Column addr set, 4 args, no delay:
	0x00, 0x00,             //     XSTART = 0
	0x00, 0x7F,             //     XEND = 127
	ST7735_SET_ROW_ADDR  , 4      ,  //  2: Row addr set, 4 args, no delay:
	0x00, 0x00,             //     XSTART = 0
	0x00, 0x7F
};           //     XEND = 127

uint8_t init_cmds3[] = {            // Init for 7735R, part 3 (red or green tab)
	4,                        //  4 commands in list:
	ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
	0x02, 0x1c, 0x07, 0x12,
	0x37, 0x32, 0x29, 0x2d,
	0x29, 0x25, 0x2B, 0x39,
	0x00, 0x01, 0x03, 0x10,
	ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
	0x03, 0x1d, 0x07, 0x06,
	0x2E, 0x2C, 0x29, 0x2D,
	0x2E, 0x2E, 0x37, 0x3F,
	0x00, 0x00, 0x02, 0x10,
	ST7735_NORON  ,    0x80, //  3: Normal display on, no args, w/delay
	10,                     //     10 ms delay
	ST7735_DISPON ,    0x80, //  4: Main screen turn on, no args w/delay
	100
};                  //     100 ms delay

typedef enum {
	SELECT_ENABLE = 0,
	SELECT_DISABLE
} select_t;

typedef void (*select_func)(st7735_hw_info_t hw_info, select_t select);

typedef struct st7735 {
	st7735_hw_info_t 	hw_info;
	st7735_size_t 		size;
	st7735_rot_t 		rot;
	uint8_t 			width;
	uint8_t				height;
	uint8_t 			cur_x;
	uint8_t 			cur_y;
	select_func 		_select;
	SemaphoreHandle_t 	lock;
} st7735_t;

static stm_err_t _write_cmd(st7735_hw_info_t hw_info, uint8_t cmd)
{
	gpio_set_level(hw_info.gpio_port_dc , hw_info.gpio_num_dc, false);
	ST7735_CHECK(!spi_write_bytes(hw_info.spi_num, &cmd, 1, TIMEOUT_MS), ST7735_WRITE_CMD_ERR_STR, return STM_FAIL);

	return STM_OK;
}

static stm_err_t _write_data(st7735_hw_info_t hw_info, uint8_t *data, uint16_t len)
{
	gpio_set_level(hw_info.gpio_port_dc , hw_info.gpio_num_dc, true);
	ST7735_CHECK(!spi_write_bytes(hw_info.spi_num, data, len, TIMEOUT_MS), ST7735_WRITE_DATA_ERR_STR, return STM_FAIL);

	return STM_OK;
}

static void _reset(st7735_hw_info_t hw_info)
{
	if ((hw_info.gpio_port_rst != -1) && (hw_info.gpio_num_rst != -1)) {
		gpio_set_level(hw_info.gpio_port_rst , hw_info.gpio_num_rst, false);
		vTaskDelay(5 / portTICK_PERIOD_MS);
		gpio_set_level(hw_info.gpio_port_rst , hw_info.gpio_num_rst, true);
	}
}

static void _write_list_cmd(st7735_hw_info_t hw_info, uint8_t *list_cmd)
{
	uint8_t num_cmd, num_arg;
	uint16_t delay_ms;

	num_cmd = *list_cmd++;
	while (num_cmd--) {
		uint8_t cmd = *list_cmd++;
		_write_cmd(hw_info, cmd);

		num_arg = *list_cmd++;
		// If high bit set, delay follows args
		delay_ms = num_arg & 0x80;
		num_arg &= ~0x80;
		if (num_arg) {
			_write_data(hw_info, list_cmd, num_arg);
			list_cmd += num_arg;
		}

		if (delay_ms) {
			delay_ms = *list_cmd++;
			if (delay_ms == 255) delay_ms = 500;
			vTaskDelay(delay_ms / portTICK_PERIOD_MS);
		}
	}
}

static void _set_addr(st7735_hw_info_t hw_info, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	uint8_t data[4];

	data[0] = 0x00;
	data[1] = x0 + ST7735_XSTART;
	data[2] = 0x00;
	data[3] = x1 + ST7735_XSTART;
	_write_cmd(hw_info, ST7735_SET_COLUMN_ADDR);
	_write_data(hw_info, data, 4);

	data[1] = y0 + ST7735_YSTART;
	data[3] = y1 + ST7735_YSTART;
	_write_cmd(hw_info, ST7735_SET_ROW_ADDR);
	_write_data(hw_info, data, 4);

	_write_cmd(hw_info, ST7735_RAMWR);
}

static void _select_with_pincs(st7735_hw_info_t hw_info, select_t select)
{
	if (select == SELECT_ENABLE) {
		gpio_set_level(hw_info.gpio_port_cs, hw_info.gpio_num_cs, false);
	} else {
		gpio_set_level(hw_info.gpio_port_cs, hw_info.gpio_num_cs, true);
	}

}

static void _select_none(st7735_hw_info_t hw_info, select_t select)
{
	/* Nothing to do */
}

static select_func _get_select_func(st7735_hw_info_t hw_info)
{
	if ((hw_info.gpio_port_cs == -1) && (hw_info.gpio_num_cs == -1)) {
		return _select_none;
	} else {
		return _select_with_pincs;
	}

	return NULL;
}

static uint8_t _get_screen_width(st7735_rot_t rot, st7735_size_t size)
{
	if ((rot == ST7735_ROT_0_DEG) || (rot == ST7735_ROT_180_DEG)) {
		if (size == ST7735_SIZE_128_128) {
			return 128;
		} else {
			return 128;
		}
	} else {
		if (size == ST7735_SIZE_128_128) {
			return 128;
		} else {
			return 160;
		}
	}
}

static uint8_t _get_screen_height(st7735_rot_t rot, st7735_size_t size)
{
	if ((rot == ST7735_ROT_0_DEG) || (rot == ST7735_ROT_180_DEG)) {
		if (size == ST7735_SIZE_128_128) {
			return 128;
		} else {
			return 160;
		}
	} else {
		if (size == ST7735_SIZE_128_128) {
			return 128;
		} else {
			return 128;
		}
	}
}

static void _clean_up(st7735_handle_t handle)
{
	free(handle);
}

st7735_handle_t st7735_init(st7735_cfg_t *config)
{
	ST7735_CHECK(config, ST7735_INIT_ERR_STR, return NULL);

	st7735_handle_t handle = calloc(1, sizeof(st7735_t));
	ST7735_CHECK(handle, ST7735_INIT_ERR_STR, return NULL);

	gpio_cfg_t gpio_cfg;
	gpio_cfg.mode = GPIO_OUTPUT_PP;
	gpio_cfg.reg_pull_mode = GPIO_REG_PULL_NONE;

	gpio_cfg.gpio_port = config->hw_info.gpio_port_rst;
	gpio_cfg.gpio_num = config->hw_info.gpio_num_rst;
	ST7735_CHECK(!gpio_config(&gpio_cfg), ST7735_INIT_ERR_STR, return NULL);

	gpio_cfg.gpio_port = config->hw_info.gpio_port_cs;
	gpio_cfg.gpio_num = config->hw_info.gpio_num_cs;
	ST7735_CHECK(!gpio_config(&gpio_cfg), ST7735_INIT_ERR_STR, return NULL);

	gpio_cfg.gpio_port = config->hw_info.gpio_port_dc;
	gpio_cfg.gpio_num = config->hw_info.gpio_num_dc;
	ST7735_CHECK(!gpio_config(&gpio_cfg), ST7735_INIT_ERR_STR, return NULL);

	if (!config->hw_info.is_init) {
		spi_cfg_t spi_cfg = {
			.spi_num = config->hw_info.spi_num,
			.spi_pins_pack = config->hw_info.spi_pins_pack,
			.mode = SPI_MODE_MASTER_HALF_DUPLEX,
			.cap_edge = SPI_CAP_FALLING_EDGE,
			.firstbit =  SPI_TRANS_FIRSTBIT_MSB
		};
		ST7735_CHECK(!spi_config(&spi_cfg), ST7735_INIT_ERR_STR, {_clean_up(handle); return NULL;});
	}

	select_func _select = _get_select_func(config->hw_info);

	_select(config->hw_info, SELECT_ENABLE);
	_reset(config->hw_info);
	_write_list_cmd(config->hw_info, init_cmds1);
	_write_list_cmd(config->hw_info, init_cmds2);
	_write_list_cmd(config->hw_info, init_cmds3);
	_select(config->hw_info, SELECT_DISABLE);

	handle->hw_info = config->hw_info;
	handle->size = config->size;
	handle->rot = config->rot;
	handle->width = _get_screen_width(config->rot, config->size);
	handle->height = _get_screen_height(config->rot, config->size);
	handle->cur_x = 0;
	handle->cur_y = 0;
	handle->_select = _get_select_func(config->hw_info);
	handle->lock = mutex_create();

	return handle;
}

stm_err_t st7735_fill_screen(st7735_handle_t handle, uint16_t color)
{
	ST7735_CHECK(handle, ST7735_FILL_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);
	handle->_select(handle->hw_info, SELECT_ENABLE);

	_set_addr(handle->hw_info, 0, 0, handle->width - 1, handle->height - 1);

	uint8_t data[2] = { color >> 8, color & 0xFF };

	for (uint8_t heigt_idx = 0; heigt_idx < handle->height; heigt_idx++) {
		for (uint8_t width_idx = 0; width_idx < handle->width; width_idx++) {
			_write_data(handle->hw_info, data, 2);
		}
	}

	handle->_select(handle->hw_info, SELECT_DISABLE);
	mutex_unlock(handle->lock);

	return STM_OK;
}

