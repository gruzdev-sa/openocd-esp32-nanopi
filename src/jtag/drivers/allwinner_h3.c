/***************************************************************************
 *   Copyright (C) 2022 by Sergey Gruzdev, gruzdev.sa@gmail.com            *
 *                                                                         *
 *   Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au *
 *   Based on at91rm9200.c (c) Anders Larsen                               *
 *   and RPi GPIO examples by Gert van Loo & Dom                           *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <transport/transport.h>
#include "bitbang.h"

#include <sys/mman.h>

/* Базовый адрес блока перефирии 0x01c20800  */
/* данный адрес мэпится в адресное пространство 0x7e000000 */
uint32_t allwinner_h3_peri_base = 0x01c20800;
#define ALLWINNER_H3_GPIO_BASE	(allwinner_h3_base) /* смещение GPIO controller */


/* GPIO setup macros */
/* Получение из регистра текущего решима работы GPIO (3 бита): 
 *
 * *(pio_base+((g)/10)) указывает на адрес регистра состояния GPIO 
 * всего есть 6 регистров, в каждом размещено по 10 GPIO, 
 * на каждый GPIO отводится 3 бита
 * т.е. по факту вычисляется адрес, где лежит регистр, 
 * после чего сдвигается вправо на кол-во бит, равное месту размещения 
 * GPIO в регистре, после чего делаеся логическая побитовая операция И (&) с числом 0b111  (7) 
 * 
 * Допустимые режимы GPIO:
 * 000 = GPIO is an input
 * 001 = GPIO is an output
 * 100 = GPIO takes alternate function 2
 * 101 = GPIO takes alternate function 3
 * 110 = GPIO takes alternate function 4
 * 111 = GPIO disabled
 * 011 = GPIO takes alternate function 1
 * 010 = GPIO takes alternate function 0
 */
/* #define MODE_GPIO(g) (*(pio_base+((g)/10))>>(((g)%10)*3) & 7) */
#define MODE_GPIO(g) (*(pio_base + ((g)/8))>>(((g)%8)*4) & 7)

/* Перевод GPIO в режим входа  (0b000)
 *
 * ~(7<<(((g)%10)*3)) - получение маски для определения места нахождения битов режима работы
 * GPIO, т.е. сначала вычисляется местоположение битов (((g)%10)*3), после чего 0b111 (7) 
 * сдвигается в лево на найденное кол-во бит, после этого результат инвертируется с помощью 
 * операции логического НЕ (~);
 * далее полученная маска с помощью операции побитового логического И складывается со значением
 * хранящемся в регистре, таким образом 3 бита текущего состоияния GPIO устанавливаются в 0b000
 *   
 */
#define INP_GPIO(g) do { *(pio_base+((g)/8)) &= ~(7<<(((g)%8)*4)); } while (0)

/* Установка режима работы GPIO в режим, указанный во втором параметре макроса (m)
 *
 * Сначала порт переводится в режим входа с помощью макроса INP_GPIO(g), далее идут операции:
 * 
 * ((m)<<(((g)%10)*3)) - получение маски для определения места нахождения битов режима работы
 * GPIO, т.е. сначала вычисляется местоположение битов (((g)%10)*3), после чего новый режим работы (m) 
 * сдвигается в лево на найденное кол-во бит.
 * далее полученная маска с помощью операции побитового логического ИЛИ складывается со значением
 * хранящемся в регистре, таким образом 3 бита текущего состоияния GPIO устанавливаются в (m) 
 */
#define SET_MODE_GPIO(g, m) do { /* clear the mode bits first, then set as necessary */ \
		INP_GPIO(g);						\
		*(pio_base+((g)/8)) |=  ((m)<<(((g)%8)*4)); } while (0)

		
#define OUT_GPIO(g) SET_MODE_GPIO(g, 1)

/*

#define GPIO_SET (*(pio_base+7))  /* sets   bits which are 1, ignores bits which are 0  адрес 0x2020001с
#define GPIO_CLR (*(pio_base+10)) /* clears bits which are 1, ignores bits which are 0  адрес 0x20200028
#define GPIO_LEV (*(pio_base+13)) /* current level of the pin адрес 0x20200034

*/

static int dev_mem_fd;
static volatile uint32_t *pio_base;

static bb_value_t allwinner_h3gpio_read(void);
static int allwinner_h3gpio_write(int tck, int tms, int tdi);

static int allwinner_h3_swdio_read(void);
static void allwinner_h3_swdio_drive(bool is_output);
static int allwinner_h3gpio_swd_write(int swclk, int swdio);

static int allwinner_h3gpio_init(void);
static int allwinner_h3gpio_quit(void);

static struct bitbang_interface allwinner_h3gpio_bitbang = {
	.read = allwinner_h3gpio_read,
	.write = allwinner_h3gpio_write,
	.swdio_read = allwinner_h3_swdio_read,
	.swdio_drive = allwinner_h3_swdio_drive,
	.swd_write = allwinner_h3gpio_swd_write,
	.blink = NULL
};

/* GPIO numbers for each signal. Negative values are invalid */
static int tck_gpio = -1;
static int tck_gpio_mode;
static int tms_gpio = -1;
static int tms_gpio_mode;
static int tdi_gpio = -1;
static int tdi_gpio_mode;
static int tdo_gpio = -1;
static int tdo_gpio_mode;
static int trst_gpio = -1;
static int trst_gpio_mode;
static int srst_gpio = -1;
static int srst_gpio_mode;
static int swclk_gpio = -1;
static int swclk_gpio_mode;
static int swdio_gpio = -1;
static int swdio_gpio_mode;
static int swdio_dir_gpio = -1;
static int swdio_dir_gpio_mode;

/* Transition delay coefficients */
static int speed_coeff = 113714;
static int speed_offset = 28;
static unsigned int jtag_delay;

static bb_value_t allwinner_h3gpio_read(void)
{
	return (GPIO_LEV & 1<<tdo_gpio) ? BB_HIGH : BB_LOW;
}

static int allwinner_h3gpio_write(int tck, int tms, int tdi)
{
	uint32_t set = tck<<tck_gpio | tms<<tms_gpio | tdi<<tdi_gpio;
	uint32_t clear = !tck<<tck_gpio | !tms<<tms_gpio | !tdi<<tdi_gpio;

	GPIO_SET = set;
	GPIO_CLR = clear;

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

static int allwinner_h3gpio_swd_write(int swclk, int swdio)
{
	uint32_t set = swclk << swclk_gpio | swdio << swdio_gpio;
	uint32_t clear = !swclk << swclk_gpio | !swdio << swdio_gpio;

	GPIO_SET = set;
	GPIO_CLR = clear;

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int allwinner_h3gpio_reset(int trst, int srst)
{
	uint32_t set = 0;
	uint32_t clear = 0;

	if (trst_gpio > 0) {
		set |= !trst<<trst_gpio;
		clear |= trst<<trst_gpio;
	}

	if (srst_gpio > 0) {
		set |= !srst<<srst_gpio;
		clear |= srst<<srst_gpio;
	}

	GPIO_SET = set;
	GPIO_CLR = clear;

	return ERROR_OK;
}

static void allwinner_h3_swdio_drive(bool is_output)
{
	if (swdio_dir_gpio > 0) {
		if (is_output) {
			GPIO_SET = 1 << swdio_dir_gpio;
			OUT_GPIO(swdio_gpio);
		} else {
			INP_GPIO(swdio_gpio);
			GPIO_CLR = 1 << swdio_dir_gpio;
		}
	} else {
		if (is_output)
			OUT_GPIO(swdio_gpio);
		else
			INP_GPIO(swdio_gpio);
	}
}

static int allwinner_h3_swdio_read(void)
{
	return !!(GPIO_LEV & 1 << swdio_gpio);
}

static int allwinner_h3gpio_khz(int khz, int *jtag_speed)
{
	if (!khz) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}
	*jtag_speed = speed_coeff/khz - speed_offset;
	if (*jtag_speed < 0)
		*jtag_speed = 0;
	return ERROR_OK;
}

static int allwinner_h3gpio_speed_div(int speed, int *khz)
{
	*khz = speed_coeff/(speed + speed_offset);
	return ERROR_OK;
}

static int allwinner_h3gpio_speed(int speed)
{
	jtag_delay = speed;
	return ERROR_OK;
}

static int is_gpio_valid(int gpio)
{
	return gpio >= 0 && gpio <= 31;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_jtag_gpionums)
{
	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], tms_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], tdi_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], tdo_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"allwinner_h3 GPIO config: tck = %d, tms = %d, tdi = %d, tdo = %d",
			tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);

	command_print(CMD, "allwinner_h3 GPIO config: tck = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);

	command_print(CMD, "allwinner_h3 GPIO config: tms = %d", tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);

	command_print(CMD, "allwinner_h3 GPIO config: tdo = %d", tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);

	command_print(CMD, "allwinner_h3 GPIO config: tdi = %d", tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD, "allwinner_h3 GPIO config: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD, "allwinner_h3 GPIO config: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_swd_gpionums)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], swdio_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"allwinner_h3 GPIO nums: swclk = %d, swdio = %d",
			swclk_gpio, swdio_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_swd_gpionum_swclk)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);

	command_print(CMD, "allwinner_h3 num: swclk = %d", swclk_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_swd_gpionum_swdio)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swdio_gpio);

	command_print(CMD, "allwinner_h3 num: swdio = %d", swdio_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_swd_dir_gpionum_swdio)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swdio_dir_gpio);

	command_print(CMD, "allwinner_h3 num: swdio_dir = %d", swdio_dir_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_speed_coeffs)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);
	}

	command_print(CMD, "allwinner_h3 GPIO: speed_coeffs = %d, speed_offset = %d",
				  speed_coeff, speed_offset);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_h3gpio_handle_peripheral_base)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], allwinner_h3_peri_base);

	command_print(CMD, "allwinner_h3 GPIO: peripheral_base = 0x%08x",
				  allwinner_h3_peri_base);
	return ERROR_OK;
}

static const struct command_registration allwinner_h3gpio_subcommand_handlers[] = {
	{
		.name = "jtag_nums",
		.handler = &allwinner_h3gpio_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "tck_num",
		.handler = &allwinner_h3gpio_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "[tck]",
	},
	{
		.name = "tms_num",
		.handler = &allwinner_h3gpio_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "[tms]",
	},
	{
		.name = "tdo_num",
		.handler = &allwinner_h3gpio_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "[tdo]",
	},
	{
		.name = "tdi_num",
		.handler = &allwinner_h3gpio_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "[tdi]",
	},
	{
		.name = "swd_nums",
		.handler = &allwinner_h3gpio_handle_swd_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for swclk, swdio. (in that order)",
		.usage = "[swclk swdio]",
	},
	{
		.name = "swclk_num",
		.handler = &allwinner_h3gpio_handle_swd_gpionum_swclk,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swclk.",
		.usage = "[swclk]",
	},
	{
		.name = "swdio_num",
		.handler = &allwinner_h3gpio_handle_swd_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio.",
		.usage = "[swdio]",
	},
	{
		.name = "swdio_dir_num",
		.handler = &allwinner_h3gpio_handle_swd_dir_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio direction control pin (set=output mode, clear=input mode)",
		.usage = "[swdio_dir]",
	},
	{
		.name = "srst_num",
		.handler = &allwinner_h3gpio_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "trst_num",
		.handler = &allwinner_h3gpio_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
	},
	{
		.name = "speed_coeffs",
		.handler = &allwinner_h3gpio_handle_speed_coeffs,
		.mode = COMMAND_CONFIG,
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",
		.usage = "[SPEED_COEFF SPEED_OFFSET]",
	},
	{
		.name = "peripheral_base",
		.handler = &allwinner_h3gpio_handle_peripheral_base,
		.mode = COMMAND_CONFIG,
		.help = "peripheral base to access GPIOs (RPi1 0x20000000, RPi2 0x3F000000).",
		.usage = "[base]",
	},

	COMMAND_REGISTRATION_DONE
};

static const struct command_registration allwinner_h3gpio_command_handlers[] = {
	{
		.name = "allwinner_h3gpio",
		.mode = COMMAND_ANY,
		.help = "perform allwinner_h3gpio management",
		.chain = allwinner_h3gpio_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const char * const allwinner_h3_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface allwinner_h3gpio_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver allwinner_h3gpio_adapter_driver = {
	.name = "allwinner_h3gpio",
	.transports = allwinner_h3_transports,
	.commands = allwinner_h3gpio_command_handlers,

	.init = allwinner_h3gpio_init,
	.quit = allwinner_h3gpio_quit,
	.reset = allwinner_h3gpio_reset,
	.speed = allwinner_h3gpio_speed,
	.khz = allwinner_h3gpio_khz,
	.speed_div = allwinner_h3gpio_speed_div,

	.jtag_ops = &allwinner_h3gpio_interface,
	.swd_ops = &bitbang_swd,
};

static bool allwinner_h3gpio_jtag_mode_possible(void)
{
	if (!is_gpio_valid(tck_gpio))
		return 0;
	if (!is_gpio_valid(tms_gpio))
		return 0;
	if (!is_gpio_valid(tdi_gpio))
		return 0;
	if (!is_gpio_valid(tdo_gpio))
		return 0;
	return 1;
}

static bool allwinner_h3gpio_swd_mode_possible(void)
{
	if (!is_gpio_valid(swclk_gpio))
		return 0;
	if (!is_gpio_valid(swdio_gpio))
		return 0;
	return 1;
}

static int allwinner_h3gpio_init(void)
{
	bitbang_interface = &allwinner_h3gpio_bitbang;

	LOG_INFO("allwinner_h3 GPIO JTAG/SWD bitbang driver");

	if (transport_is_jtag() && !allwinner_h3gpio_jtag_mode_possible()) {
		LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (transport_is_swd() && !allwinner_h3gpio_swd_mode_possible()) {
		LOG_ERROR("Require swclk and swdio gpio for SWD mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	dev_mem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
	if (dev_mem_fd < 0) {
		LOG_DEBUG("Cannot open /dev/gpiomem, fallback to /dev/mem");
		dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	}
	if (dev_mem_fd < 0) {
		LOG_ERROR("open: %s", strerror(errno));
		return ERROR_JTAG_INIT_FAILED;
	}

	pio_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, allwinner_h3_GPIO_BASE);

	if (pio_base == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	/* Установка оганичения по току на GPI0 - GPI27
	 * на 4 mA
	 * + включается гистерезис (защита от дребезга контактов)
	 */
	static volatile uint32_t *pads_base;
	pads_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, allwinner_h3_PADS_GPIO_0_27);

	if (pads_base == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	/* set 4mA drive strength, slew rate limited, hysteresis on */
	pads_base[allwinner_h3_PADS_GPIO_0_27_OFFSET] = 0x5a000008 + 1;
	/* Выше - установка оганичения по току на GPI0 - GPI27
	 * на 4 mA
	 * + включается гистерезис (защита от дребезга контактов)
	 */

	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST
	 * as outputs.  Drive TDI and TCK low, and TMS/TRST/SRST high.
	 */
	if (transport_is_jtag()) {
		tdo_gpio_mode = MODE_GPIO(tdo_gpio);
		tdi_gpio_mode = MODE_GPIO(tdi_gpio);
		tck_gpio_mode = MODE_GPIO(tck_gpio);
		tms_gpio_mode = MODE_GPIO(tms_gpio);

		INP_GPIO(tdo_gpio);

		GPIO_CLR = 1<<tdi_gpio | 1<<tck_gpio;
		GPIO_SET = 1<<tms_gpio;

		OUT_GPIO(tdi_gpio);
		OUT_GPIO(tck_gpio);
		OUT_GPIO(tms_gpio);

		if (trst_gpio != -1) {
			trst_gpio_mode = MODE_GPIO(trst_gpio);
			GPIO_SET = 1 << trst_gpio;
			OUT_GPIO(trst_gpio);
		}
	}

	if (transport_is_swd()) {
		swclk_gpio_mode = MODE_GPIO(swclk_gpio);
		swdio_gpio_mode = MODE_GPIO(swdio_gpio);

		GPIO_CLR = 1<<swdio_gpio | 1<<swclk_gpio;

		OUT_GPIO(swclk_gpio);
		OUT_GPIO(swdio_gpio);
	}

	if (srst_gpio != -1) {
		srst_gpio_mode = MODE_GPIO(srst_gpio);
		GPIO_SET = 1 << srst_gpio;
		OUT_GPIO(srst_gpio);
	}

	if (swdio_dir_gpio != -1) {
		swdio_dir_gpio_mode = MODE_GPIO(swdio_dir_gpio);
		GPIO_SET = 1 << swdio_dir_gpio;
		OUT_GPIO(swdio_dir_gpio);
	}

	LOG_DEBUG("saved pinmux settings: tck %d tms %d tdi %d "
		  "tdo %d trst %d srst %d", tck_gpio_mode, tms_gpio_mode,
		  tdi_gpio_mode, tdo_gpio_mode, trst_gpio_mode, srst_gpio_mode);

	return ERROR_OK;
}

static int allwinner_h3gpio_quit(void)
{
	if (transport_is_jtag()) {
		SET_MODE_GPIO(tdo_gpio, tdo_gpio_mode);
		SET_MODE_GPIO(tdi_gpio, tdi_gpio_mode);
		SET_MODE_GPIO(tck_gpio, tck_gpio_mode);
		SET_MODE_GPIO(tms_gpio, tms_gpio_mode);
		if (trst_gpio != -1)
			SET_MODE_GPIO(trst_gpio, trst_gpio_mode);
	}

	if (transport_is_swd()) {
		SET_MODE_GPIO(swclk_gpio, swclk_gpio_mode);
		SET_MODE_GPIO(swdio_gpio, swdio_gpio_mode);
	}

	if (srst_gpio != -1)
		SET_MODE_GPIO(srst_gpio, srst_gpio_mode);

	if (swdio_dir_gpio != -1)
		SET_MODE_GPIO(swdio_dir_gpio, swdio_dir_gpio_mode);

	return ERROR_OK;
}
