// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2024 Vestas Wind Systems A/S                            *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <transport/transport.h>
#include "bitbang.h"

#include <sys/mman.h>

/* 32-bit register offsets (see PG144 October 5, 2016) */
#define XLNX_AXI_GPIO_SIZE 0x1000
#define XLNX_AXI_GPIO_DATA 0x0000
#define XLNX_AXI_GPIO_TRI  0x0001

static const struct adapter_gpio_config *adapter_gpio_config;
uint32_t xlnx_axi_gpio_peri_base = 0;
static int dev_mem_fd;
static uint32_t *xlnx_axi_gpio_base;
static volatile uint32_t *xlnx_axi_gpio_reg_data;
static volatile uint32_t *xlnx_axi_gpio_reg_tri;

bb_value_t xlnx_axi_gpio_read(void)
{
	const uint32_t mask = BIT(adapter_gpio_config[ADAPTER_GPIO_IDX_TDO].gpio_num);

	return *xlnx_axi_gpio_reg_data & mask ? BB_HIGH : BB_LOW;
}

static int xlnx_axi_gpio_write(int tck, int tms, int tdi)
{
	uint32_t data = 0;

	if (tck)
		data |= BIT(adapter_gpio_config[ADAPTER_GPIO_IDX_TCK].gpio_num);

	if (tms)
		data |= BIT(adapter_gpio_config[ADAPTER_GPIO_IDX_TMS].gpio_num);

	if (tdi)
		data |= BIT(adapter_gpio_config[ADAPTER_GPIO_IDX_TDI].gpio_num);

	*xlnx_axi_gpio_reg_data = data;

	return ERROR_OK;
}

static int xlnx_axi_gpio_reset(int trst, int srst)
{
	/* TODO: add reset support */

	return ERROR_OK;
}

COMMAND_HANDLER(xlnx_axi_gpio_handle_peripheral_base)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], xlnx_axi_gpio_peri_base);

	command_print(CMD, "Xilinx AXI GPIO: peripheral_base = 0x%08x", xlnx_axi_gpio_peri_base);
	return ERROR_OK;
}

static const struct command_registration xlnx_axi_gpio_subcommand_handlers[] = {
	{
		.name = "peripheral_base",
		.handler = &xlnx_axi_gpio_handle_peripheral_base,
		.mode = COMMAND_CONFIG,
		.help = "peripheral base to access GPIOs.",
		.usage = "[base]",
	},

	COMMAND_REGISTRATION_DONE
};

static const struct command_registration xlnx_axi_gpio_command_handlers[] = {
	/* TODO: speed? */
	{
		.name = "xlnx_axi_gpio",
		.mode = COMMAND_ANY,
		.help = "perform xlnx_axi_gpio management",
		.chain = xlnx_axi_gpio_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static bool is_gpio_config_valid(enum adapter_gpio_config_index idx)
{
	/* TODO: support GPIO2 using chip_num? */
	return adapter_gpio_config[idx].chip_num >= -1
		&& adapter_gpio_config[idx].chip_num <= 0
		&& adapter_gpio_config[idx].gpio_num >= 0
		&& adapter_gpio_config[idx].gpio_num <= 31;
}

static bool xlnx_axi_gpio_jtag_mode_possible(void)
{
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TCK))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TMS))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TDI))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TDO))
		return false;
	return true;
}

int xlnx_axi_gpio_quit(void)
{
	/* TODO: pin clean-up needed? */

	munmap((void *)xlnx_axi_gpio_base, XLNX_AXI_GPIO_SIZE);
	close(dev_mem_fd);

	return ERROR_OK;
}


static struct bitbang_interface xlnx_axi_gpio_bitbang = {
	.read = xlnx_axi_gpio_read,
	.write = xlnx_axi_gpio_write,
	.blink = 0,
};

int xlnx_axi_gpio_init(void)
{
	uint32_t data = 0;
	uint32_t tri = 0;

	LOG_INFO("Xilinx AXI GPIO JTAG bitbang driver");

	bitbang_interface = &xlnx_axi_gpio_bitbang;
	adapter_gpio_config = adapter_gpio_get_config();

	dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_mem_fd < 0) {
		LOG_ERROR("open: %s", strerror(errno));
		return ERROR_JTAG_INIT_FAILED;
	}

	xlnx_axi_gpio_base = mmap(NULL, XLNX_AXI_GPIO_SIZE, PROT_READ | PROT_WRITE,
				  MAP_SHARED, dev_mem_fd, xlnx_axi_gpio_peri_base);

	if (xlnx_axi_gpio_base == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		goto out_error;
	}

	xlnx_axi_gpio_reg_data = xlnx_axi_gpio_base + XLNX_AXI_GPIO_DATA;
	xlnx_axi_gpio_reg_tri = xlnx_axi_gpio_base + XLNX_AXI_GPIO_TRI;

	if (transport_is_jtag()) {
		if (!xlnx_axi_gpio_jtag_mode_possible()) {
			LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode");
			goto out_error;
		}

		LOG_INFO("peripheral_base = 0x%08x", xlnx_axi_gpio_peri_base);

		LOG_INFO("tck chip_num = %d, gpio_num = %d",
			adapter_gpio_config[ADAPTER_GPIO_IDX_TCK].chip_num,
			adapter_gpio_config[ADAPTER_GPIO_IDX_TCK].gpio_num);
		LOG_INFO("tdo chip_num = %d, gpio_num = %d",
			adapter_gpio_config[ADAPTER_GPIO_IDX_TDO].chip_num,
			adapter_gpio_config[ADAPTER_GPIO_IDX_TDO].gpio_num);
		LOG_INFO("tms chip_num = %d, gpio_num = %d",
			adapter_gpio_config[ADAPTER_GPIO_IDX_TMS].chip_num,
			adapter_gpio_config[ADAPTER_GPIO_IDX_TMS].gpio_num);
		LOG_INFO("tdi chip_num = %d, gpio_num = %d",
			adapter_gpio_config[ADAPTER_GPIO_IDX_TDI].chip_num,
			adapter_gpio_config[ADAPTER_GPIO_IDX_TDI].gpio_num);

		/* Configure TDO as input */
		tri |= BIT(adapter_gpio_config[ADAPTER_GPIO_IDX_TDO].gpio_num);
		*xlnx_axi_gpio_reg_tri = tri;

		/* Set TMS */
		data |= BIT(adapter_gpio_config[ADAPTER_GPIO_IDX_TMS].gpio_num);
		*xlnx_axi_gpio_reg_data = data;
	}

	return ERROR_OK;

out_error:
	xlnx_axi_gpio_quit();

	return ERROR_JTAG_INIT_FAILED;
}

static struct jtag_interface xlnx_axi_gpio_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver xlnx_axi_gpio_adapter_driver = {
	.name = "xlnx_axi_gpio",
	.transports = jtag_only,
	.commands = xlnx_axi_gpio_command_handlers,

	.init = xlnx_axi_gpio_init,
	.quit = xlnx_axi_gpio_quit,
	.reset = xlnx_axi_gpio_reset,

	.jtag_ops = &xlnx_axi_gpio_interface,
};
