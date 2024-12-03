/* SPDX-License-Identifier: GPL-2.0
 *
 *	 Copyright (C) 2019 Google, LLC.
 *	 Moritz Fischer <moritzf@google.com>
 *
 *	 Copyright (C) 2021 Western Digital Corporation or its affiliates
 *	 Jeremy Garff <jeremy.garff@wdc.com>
 */

/*
 * This implementation was derived from the PCIe Xilinx Debug Bridge driver
 * found in xlnx-pcie-xvc.c.  Instead of using PCIe, this driver is used
 * for direct AXI connections.	This is useful where OpenOCD is running in
 * a Zynq or on a MicroBlaze with direct access to the register interface.
 * of the debug bridge.
 *
 * Future enhancements should include combining this with the PCIe driver
 * to optimize use of common code and reduce duplication.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <linux/pci.h>
#include <sys/mman.h>

#include <jtag/interface.h>
#include <jtag/swd.h>
#include <jtag/commands.h>
#include <helper/replacements.h>
#include <helper/bits.h>

#include <gpiod.h>

#define XLNX_XVC_LEN_REG				 0x00
#define XLNX_XVC_TMS_REG				 0x04
#define XLNX_XVC_TDI_REG				 0x08
#define XLNX_XVC_TDO_REG				 0x0c
#define XLNX_XVC_CTRL_REG				 0x10
/* #define XLNX_XVC_DIV_REG				 0x14		/\* Non-standard register added to Xilinx HDL -- JG *\/ */
/* #define XLNX_XVC_MAX_REG				 0x18 */
#define XLNX_XVC_MAX_REG				 0x14

#define XLNX_XVC_CTRL_REG_ENABLE_MASK	 0x01

#define XLNX_XVC_BASE_FREQ				 100000000	/* 100Mhz */

#define XLNX_XVC_MAX_BITS				 0x20

#define MASK_ACK(x) (((x) >> 9) & 0x7)
#define MASK_PAR(x) ((int)((x) & 0x1))


struct xlnx_axi_signal {
	char *name;
	struct gpiod_line *line;
};

struct xlnx_axi_xvc {
	int fd;
	uint32_t *baseaddr;
	char *deviceaddr;
	struct xlnx_axi_signal trst;
	struct xlnx_axi_signal srst;
};

static struct xlnx_axi_xvc xlnx_axi_xvc_state;
static struct xlnx_axi_xvc *xlnx_axi_xvc = &xlnx_axi_xvc_state;

static int xlnx_axi_xvc_read_reg(const int offset, uint32_t *val)
{
	uint8_t *b = (uint8_t *)xlnx_axi_xvc->baseaddr + offset;
	volatile uint32_t *w = (uint32_t *)b;

	if (val)
		*val = *w;

	return ERROR_OK;
}

static int xlnx_axi_xvc_write_reg(const int offset, const uint32_t val)
{
	uint8_t *b = (uint8_t *)xlnx_axi_xvc->baseaddr + offset;
	volatile uint32_t *w = (uint32_t *)b;

	*w = val;

	return ERROR_OK;
}

static int xlnx_axi_xvc_transact(size_t num_bits, uint32_t tms, uint32_t tdi,
				  uint32_t *tdo)
{
	uint32_t ctrl;
	int done = 0;
	int err;

	err = xlnx_axi_xvc_write_reg(XLNX_XVC_LEN_REG, num_bits);
	if (err != ERROR_OK)
		return err;

	err = xlnx_axi_xvc_write_reg(XLNX_XVC_TMS_REG, tms);
	if (err != ERROR_OK)
		return err;

	err = xlnx_axi_xvc_write_reg(XLNX_XVC_TDI_REG, tdi);
	if (err != ERROR_OK)
		return err;

	err = xlnx_axi_xvc_write_reg(XLNX_XVC_CTRL_REG, XLNX_XVC_CTRL_REG_ENABLE_MASK);
	if (err != ERROR_OK)
		return err;

	while (!done) {
		err = xlnx_axi_xvc_read_reg(XLNX_XVC_CTRL_REG, &ctrl);
		if (err != ERROR_OK)
			return err;

		if (!(ctrl & XLNX_XVC_CTRL_REG_ENABLE_MASK))
			done = 1;

		/*
		 There is no delay here intentionally.	The usleep()
		 function doesn't block and burns CPU cycles anyway.
		 The turnaround time is fast enough at high JTAG rates
		 that adding the call can slow down the overall
		 throughput.	So we'll just sacrifice the CPU to get
		 best performance.

		 Additionally there is no timeout.	The underlying
		 hardware is guaranteed to unset the enable bit within
		 32 JTAG clock cycles.	There is no hardware condition
		 that will keep it set forever.  Essentially, the hardware
		 is also our timeout mechanism.
		*/
	}

	err = xlnx_axi_xvc_read_reg(XLNX_XVC_TDO_REG, tdo);
	if (err != ERROR_OK)
		return err;

	if (tdo)
		LOG_DEBUG_IO("Transact num_bits: %zu, tms: %x, tdi: %x, tdo: %x",
				 num_bits, tms, tdi, *tdo);
	else
		LOG_DEBUG_IO("Transact num_bits: %zu, tms: %x, tdi: %x, tdo: <null>",
				 num_bits, tms, tdi);
	return ERROR_OK;
}

int xlnx_axi_xvc_execute_stableclocks(struct jtag_command *cmd)
{
	int tms = tap_get_state() == TAP_RESET ? 1 : 0;
	size_t left = cmd->cmd.stableclocks->num_cycles;
	size_t write;
	int err;

	LOG_DEBUG_IO("stableclocks %i cycles", cmd->cmd.runtest->num_cycles);

	while (left) {
		write = MIN(XLNX_XVC_MAX_BITS, left);
		err = xlnx_axi_xvc_transact(write, tms, 0, NULL);
		if (err != ERROR_OK)
			return err;
		left -= write;
	};

	return ERROR_OK;
}

static int xlnx_axi_xvc_execute_statemove(size_t skip)
{
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(),
						tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(),
						 tap_get_end_state());
	int err;

	LOG_DEBUG_IO("statemove starting at (skip: %zu) %s end in %s", skip,
		  tap_state_name(tap_get_state()),
		  tap_state_name(tap_get_end_state()));

	err = xlnx_axi_xvc_transact(tms_count - skip, tms_scan >> skip, 0, NULL);
	if (err != ERROR_OK)
		return err;

	tap_set_state(tap_get_end_state());

	return ERROR_OK;
}

static int xlnx_axi_xvc_execute_runtest(struct jtag_command *cmd)
{
	int err = ERROR_OK;

	LOG_DEBUG_IO("runtest %i cycles, end in %i",
		  cmd->cmd.runtest->num_cycles,
		  cmd->cmd.runtest->end_state);

	tap_state_t tmp_state = tap_get_end_state();

	if (tap_get_state() != TAP_IDLE) {
		tap_set_end_state(TAP_IDLE);
		err = xlnx_axi_xvc_execute_statemove(0);
		if (err != ERROR_OK)
			return err;
	};

	size_t left = cmd->cmd.runtest->num_cycles;
	size_t write;

	while (left) {
		write = MIN(XLNX_XVC_MAX_BITS, left);
		err = xlnx_axi_xvc_transact(write, 0, 0, NULL);
		if (err != ERROR_OK)
			return err;
		left -= write;
	};

	tap_set_end_state(tmp_state);
	if (tap_get_state() != tap_get_end_state())
		err = xlnx_axi_xvc_execute_statemove(0);

	return err;
}

static int xlnx_axi_xvc_execute_pathmove(struct jtag_command *cmd)
{
	size_t num_states = cmd->cmd.pathmove->num_states;
	tap_state_t *path = cmd->cmd.pathmove->path;
	int err = ERROR_OK;
	size_t i;

	LOG_DEBUG_IO("pathmove: %i states, end in %i",
		  cmd->cmd.pathmove->num_states,
		  cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

	for (i = 0; i < num_states; i++) {
		if (path[i] == tap_state_transition(tap_get_state(), false)) {
			err = xlnx_axi_xvc_transact(1, 1, 0, NULL);
		} else if (path[i] == tap_state_transition(tap_get_state(), true)) {
			err = xlnx_axi_xvc_transact(1, 0, 0, NULL);
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition.",
				  tap_state_name(tap_get_state()),
				  tap_state_name(path[i]));
			err = ERROR_JTAG_QUEUE_FAILED;
		}
		if (err != ERROR_OK)
			return err;
		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());

	return ERROR_OK;
}

static int xlnx_axi_xvc_execute_scan(struct jtag_command *cmd)
{
	enum scan_type type = jtag_scan_type(cmd->cmd.scan);
	tap_state_t saved_end_state = cmd->cmd.scan->end_state;
	bool ir_scan = cmd->cmd.scan->ir_scan;
	uint32_t tdi, tms, tdo;
	uint8_t *buf, *rd_ptr;
	int err, scan_size;
	size_t write;
	size_t left;

	scan_size = jtag_build_buffer(cmd->cmd.scan, &buf);
	rd_ptr = buf;
	LOG_DEBUG_IO("%s scan type %d %d bits; starts in %s end in %s",
		  (cmd->cmd.scan->ir_scan) ? "IR" : "DR", type, scan_size,
		  tap_state_name(tap_get_state()),
		  tap_state_name(cmd->cmd.scan->end_state));

	/* If we're in TAP_DR_SHIFT state but need to do a IR_SCAN or
	 * vice-versa, do a statemove to corresponding other state, then restore
	 * end state
	 */
	if (ir_scan && tap_get_state() != TAP_IRSHIFT) {
		tap_set_end_state(TAP_IRSHIFT);
		err = xlnx_axi_xvc_execute_statemove(0);
		if (err != ERROR_OK)
			goto out_err;
		tap_set_end_state(saved_end_state);
	} else if (!ir_scan && (tap_get_state() != TAP_DRSHIFT)) {
		tap_set_end_state(TAP_DRSHIFT);
		err = xlnx_axi_xvc_execute_statemove(0);
		if (err != ERROR_OK)
			goto out_err;
		tap_set_end_state(saved_end_state);
	}

	left = scan_size;
	while (left) {
		write = MIN(XLNX_XVC_MAX_BITS, left);
		/* the last TMS should be a 1, to leave the state */
		tms = left <= XLNX_XVC_MAX_BITS ? BIT(write - 1) : 0;
		tdi = (type != SCAN_IN) ? buf_get_u32(rd_ptr, 0, write) : 0;
		err = xlnx_axi_xvc_transact(write, tms, tdi, type != SCAN_OUT ?
						 &tdo : NULL);
		if (err != ERROR_OK)
			goto out_err;
		left -= write;
		if (type != SCAN_OUT)
			buf_set_u32(rd_ptr, 0, write, tdo);
		rd_ptr += sizeof(uint32_t);
	};

	err = jtag_read_buffer(buf, cmd->cmd.scan);
	if (buf)
		free(buf);

	if (tap_get_state() != tap_get_end_state())
		err = xlnx_axi_xvc_execute_statemove(1);

	return err;

out_err:
	if (buf)
		free(buf);
	return err;
}

static void xlnx_axi_xvc_execute_reset(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("reset trst: %i srst: %i", cmd->cmd.reset->trst,
			  cmd->cmd.reset->srst);

	if (xlnx_axi_xvc->srst.line) {
		gpiod_line_set_value(xlnx_axi_xvc->srst.line,
							 cmd->cmd.reset->srst ? 0 : 1);
	}

	if (xlnx_axi_xvc->trst.line) {
		if (cmd->cmd.reset->trst)
			tap_set_state(TAP_RESET);

		gpiod_line_set_value(xlnx_axi_xvc->trst.line,
							 cmd->cmd.reset->trst ? 0 : 1);
	}
}

static void xlnx_axi_xvc_execute_sleep(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("sleep %" PRIi32 "", cmd->cmd.sleep->us);
	usleep(cmd->cmd.sleep->us);
}

static int xlnx_axi_xvc_execute_tms(struct jtag_command *cmd)
{
	const size_t num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;
	size_t left, write;
	uint32_t tms;
	int err;

	LOG_DEBUG_IO("execute tms %zu", num_bits);

	left = num_bits;
	while (left) {
		write = MIN(XLNX_XVC_MAX_BITS, left);
		tms = buf_get_u32(bits, 0, write);
		err = xlnx_axi_xvc_transact(write, tms, 0, NULL);
		if (err != ERROR_OK)
			return err;
		left -= write;
		bits += 4;
	};

	return ERROR_OK;
}

static int xlnx_axi_xvc_execute_command(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("%s: cmd->type: %u", __func__, cmd->type);
	switch (cmd->type) {
	case JTAG_STABLECLOCKS:
		return xlnx_axi_xvc_execute_stableclocks(cmd);
	case JTAG_RUNTEST:
		return xlnx_axi_xvc_execute_runtest(cmd);
	case JTAG_TLR_RESET:
		tap_set_end_state(cmd->cmd.statemove->end_state);
		return xlnx_axi_xvc_execute_statemove(0);
	case JTAG_PATHMOVE:
		return xlnx_axi_xvc_execute_pathmove(cmd);
	case JTAG_SCAN:
		return xlnx_axi_xvc_execute_scan(cmd);
	case JTAG_RESET:
		xlnx_axi_xvc_execute_reset(cmd);
		break;
	case JTAG_SLEEP:
		xlnx_axi_xvc_execute_sleep(cmd);
		break;
	case JTAG_TMS:
		return xlnx_axi_xvc_execute_tms(cmd);
	default:
		LOG_ERROR("BUG: Unknown JTAG command type encountered.");
		return ERROR_JTAG_QUEUE_FAILED;
	}

	return ERROR_OK;
}

static int xlnx_axi_xvc_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int ret;

	while (cmd) {
		ret = xlnx_axi_xvc_execute_command(cmd);

		if (ret != ERROR_OK)
			return ret;

		cmd = cmd->next;
	}

	return ERROR_OK;
}

static int xlnx_axi_xvc_gpio_init(char *name, struct gpiod_line **line)
{
	struct gpiod_line_request_config config = {
		.request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT,
	};
	int result;

	/* Lookup the signal by name */
	*line = gpiod_line_find(name);
	if (!*line) {
		LOG_ERROR("GPIO Lookup failed for signal %s, check device-tree\n",
				  name);
		return ERROR_JTAG_INIT_FAILED;
	}

	/* Request the signal and set the direction and default value */
	result = gpiod_line_request(*line, &config, 1);
	if (result) {
		LOG_ERROR("Unable to request GPIO signals %s\n", name);
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int xlnx_axi_xvc_init(void)
{
	uint64_t baseaddr;

	if (xlnx_axi_xvc->deviceaddr) {
		baseaddr = strtoul(xlnx_axi_xvc->deviceaddr, NULL, 0);
	} else {
		LOG_ERROR("Please set deviceaddr.");
		return ERROR_JTAG_INIT_FAILED;
	}

	xlnx_axi_xvc->fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (xlnx_axi_xvc->fd < 0) {
		LOG_ERROR("Failed to open /dev/mem.  Check permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	xlnx_axi_xvc->baseaddr = mmap(0, XLNX_XVC_MAX_REG, PROT_READ | PROT_WRITE,
								  MAP_SHARED, xlnx_axi_xvc->fd, baseaddr);
	if (xlnx_axi_xvc->baseaddr == MAP_FAILED) {
		LOG_ERROR("mmap() failed.  Check permissions.");
		close(xlnx_axi_xvc->fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_INFO("Mapped Xilinx XVC/AXI vaddr %p paddr 0x%lx",
			 xlnx_axi_xvc->baseaddr, baseaddr);

	if (xlnx_axi_xvc->trst.name) {
		int result = xlnx_axi_xvc_gpio_init(xlnx_axi_xvc->trst.name,
											&xlnx_axi_xvc->trst.line);
		if (result)
			return result;

		LOG_INFO("Using signal %s for TRST", xlnx_axi_xvc->trst.name);
	}

	if (xlnx_axi_xvc->srst.name) {
		int result = xlnx_axi_xvc_gpio_init(xlnx_axi_xvc->srst.name,
											&xlnx_axi_xvc->srst.line);
		if (result)
			return result;

		LOG_INFO("Using signal %s for SRST", xlnx_axi_xvc->srst.name);
	}

	return ERROR_OK;
}

static int xlnx_axi_xvc_quit(void)
{
	int err;

	munmap(xlnx_axi_xvc->baseaddr, XLNX_XVC_MAX_REG);

	err = close(xlnx_axi_xvc->fd);
	if (err)
		return err;

	return ERROR_OK;
}

/*
static int xlnx_axi_xvc_speed(int speed)
{
	uint8_t *b = (uint8_t *)xlnx_axi_xvc->baseaddr + XLNX_XVC_DIV_REG;
	volatile uint32_t *w = (uint32_t *)b;
	int divider = XLNX_XVC_BASE_FREQ / (speed * 1000);

	if (divider <= 1) {
		LOG_ERROR("Invalid JTAG Speed (%d Khz) not supported", speed);

		return ERROR_FAIL;
	}

	if (divider > 0xffff) {
		LOG_ERROR("Invalid JTAG Speed (%d Khz) not supported", speed);

		return ERROR_FAIL;
	}

	*w = divider;

	LOG_INFO("XVC/AXI Requested JTAG Speed	 : %d Khz", speed);
	LOG_INFO("XVC/AXI Setting Clock Divider  : %d", divider);
	LOG_INFO("XVC/AXI Actual JTAG Clock Rate : %d Khz", (XLNX_XVC_BASE_FREQ / divider) / 1000);

	return ERROR_OK;
}

static int xlnx_axi_xvc_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;

	return ERROR_OK;
}

static int xlnx_axi_xvc_div(int speed, int *khz)
{
	*khz = speed;

	return ERROR_OK;
}
*/


COMMAND_HANDLER(xlnx_axi_xvc_handle_devaddr_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* we can't really free this in a safe manner, so at least
	 * limit the memory we're leaking by freeing the old one first
	 * before allocating a new one ...
	 */
	if (xlnx_axi_xvc->deviceaddr)
		free(xlnx_axi_xvc->deviceaddr);

	xlnx_axi_xvc->deviceaddr = strdup(CMD_ARGV[0]);

	return ERROR_OK;
}

COMMAND_HANDLER(xlnx_axi_xvc_handle_srst_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* we can't really free this in a safe manner, so at least
	 * limit the memory we're leaking by freeing the old one first
	 * before allocating a new one ...
	 */
	if (xlnx_axi_xvc->srst.name)
		free(xlnx_axi_xvc->srst.name);

	xlnx_axi_xvc->srst.name = strdup(CMD_ARGV[0]);

	return ERROR_OK;
}

COMMAND_HANDLER(xlnx_axi_xvc_handle_trst_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* we can't really free this in a safe manner, so at least
	 * limit the memory we're leaking by freeing the old one first
	 * before allocating a new one ...
	 */
	if (xlnx_axi_xvc->trst.name)
		free(xlnx_axi_xvc->trst.name);

	xlnx_axi_xvc->trst.name = strdup(CMD_ARGV[0]);

	return ERROR_OK;
}

static const struct command_registration xlnx_axi_xvc_command_handlers[] = {
	{
		.name = "xlnx_axi_xvc_devaddr",
		.handler = xlnx_axi_xvc_handle_devaddr_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure XVC/AXI JTAG device memory address",
		.usage = "xlnx_axi_xvc_devaddr <device address>",
	},
	{
		.name = "xlnx_axi_xvc_trst",
		.handler = xlnx_axi_xvc_handle_trst_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure XVC/AXI JTAG TRST GPIO name",
		.usage = "xlnx_axi_xvc_trst <gpio signal name>",
	},
	{
		.name = "xlnx_axi_xvc_srst",
		.handler = xlnx_axi_xvc_handle_srst_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure XVC/AXI JTAG SRST GPIOD name",
		.usage = "xlnx_axi_xvc_srst <gpio signal name>",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface xlnx_axi_xvc_jtag_ops = {
	.execute_queue = &xlnx_axi_xvc_execute_queue,
};

static int xlnx_axi_xvc_swd_sequence(const uint8_t *seq, size_t length)
{
	size_t left, write;
	uint32_t send;
	int err;

	left = length;
	while (left) {
		write = MIN(XLNX_XVC_MAX_BITS, left);
		send = buf_get_u32(seq, 0, write);
		err = xlnx_axi_xvc_transact(write, send, 0, NULL);
		if (err != ERROR_OK)
			return err;
		left -= write;
		seq += sizeof(uint32_t);
	};

	return ERROR_OK;
}

static int xlnx_axi_xvc_swd_switch_seq(enum swd_special_seq seq)
{
	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG_IO("SWD line reset");
		return xlnx_axi_xvc_swd_sequence(swd_seq_line_reset,
						  swd_seq_line_reset_len);
	case JTAG_TO_SWD:
		LOG_DEBUG_IO("JTAG-to-SWD");
		return xlnx_axi_xvc_swd_sequence(swd_seq_jtag_to_swd,
						  swd_seq_jtag_to_swd_len);
	case SWD_TO_JTAG:
		LOG_DEBUG_IO("SWD-to-JTAG");
		return xlnx_axi_xvc_swd_sequence(swd_seq_swd_to_jtag,
						  swd_seq_swd_to_jtag_len);
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int queued_retval;

static void xlnx_axi_xvc_swd_write_reg(uint8_t cmd, uint32_t value,
					uint32_t ap_delay_clk);

static void swd_clear_sticky_errors(void)
{
	xlnx_axi_xvc_swd_write_reg(swd_cmd(false,  false, DP_ABORT),
		STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR, 0);
}

static void xlnx_axi_xvc_swd_read_reg(uint8_t cmd, uint32_t *value,
					   uint32_t ap_delay_clk)
{
	uint32_t res, ack, rpar;
	int err;

	assert(cmd & SWD_CMD_RNW);

	cmd |= SWD_CMD_START | SWD_CMD_PARK;
	/* cmd + ack */
	err = xlnx_axi_xvc_transact(12, cmd, 0, &res);
	if (err != ERROR_OK)
		goto err_out;

	ack = MASK_ACK(res);

	/* read data */
	err = xlnx_axi_xvc_transact(32, 0, 0, &res);
	if (err != ERROR_OK)
		goto err_out;

	/* parity + trn */
	err = xlnx_axi_xvc_transact(2, 0, 0, &rpar);
	if (err != ERROR_OK)
		goto err_out;

	LOG_DEBUG_IO("%s %s %s reg %X = %08"PRIx32,
		  ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ?
		  "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
		  cmd & SWD_CMD_APNDP ? "AP" : "DP",
		  cmd & SWD_CMD_RNW ? "read" : "write",
		  (cmd & SWD_CMD_A32) >> 1,
		  res);
	switch (ack) {
	case SWD_ACK_OK:
		if (MASK_PAR(rpar) != parity_u32(res)) {
			LOG_DEBUG_IO("Wrong parity detected");
			queued_retval = ERROR_FAIL;
			return;
		}
		if (value)
			*value = res;
		if (cmd & SWD_CMD_APNDP)
			err = xlnx_axi_xvc_transact(ap_delay_clk, 0, 0, NULL);
		queued_retval = err;
		return;
	case SWD_ACK_WAIT:
		LOG_DEBUG_IO("SWD_ACK_WAIT");
		swd_clear_sticky_errors();
		return;
	case SWD_ACK_FAULT:
		LOG_DEBUG_IO("SWD_ACK_FAULT");
		queued_retval = ack;
		return;
	default:
		LOG_DEBUG_IO("No valid acknowledge: ack=%02"PRIx32, ack);
		queued_retval = ack;
		return;
	}
err_out:
	queued_retval = err;
}

static void xlnx_axi_xvc_swd_write_reg(uint8_t cmd, uint32_t value,
					uint32_t ap_delay_clk)
{
	uint32_t res, ack;
	int err;

	assert(!(cmd & SWD_CMD_RNW));

	cmd |= SWD_CMD_START | SWD_CMD_PARK;
	/* cmd + trn + ack */
	err = xlnx_axi_xvc_transact(13, cmd, 0, &res);
	if (err != ERROR_OK)
		goto err_out;

	ack = MASK_ACK(res);

	/* write data */
	err = xlnx_axi_xvc_transact(32, value, 0, NULL);
	if (err != ERROR_OK)
		goto err_out;

	/* parity + trn */
	err = xlnx_axi_xvc_transact(2, parity_u32(value), 0, NULL);
	if (err != ERROR_OK)
		goto err_out;

	LOG_DEBUG_IO("%s %s %s reg %X = %08"PRIx32,
		  ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ?
		  "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
		  cmd & SWD_CMD_APNDP ? "AP" : "DP",
		  cmd & SWD_CMD_RNW ? "read" : "write",
		  (cmd & SWD_CMD_A32) >> 1,
		  value);

	switch (ack) {
	case SWD_ACK_OK:
		if (cmd & SWD_CMD_APNDP)
			err = xlnx_axi_xvc_transact(ap_delay_clk, 0, 0, NULL);
		queued_retval = err;
		return;
	case SWD_ACK_WAIT:
		LOG_DEBUG_IO("SWD_ACK_WAIT");
		swd_clear_sticky_errors();
		return;
	case SWD_ACK_FAULT:
		LOG_DEBUG_IO("SWD_ACK_FAULT");
		queued_retval = ack;
		return;
	default:
		LOG_DEBUG_IO("No valid acknowledge: ack=%02"PRIx32, ack);
		queued_retval = ack;
		return;
	}

err_out:
	queued_retval = err;
}

static int xlnx_axi_xvc_swd_run_queue(void)
{
	int err;

	/* we want at least 8 idle cycles between each transaction */
	err = xlnx_axi_xvc_transact(8, 0, 0, NULL);
	if (err != ERROR_OK)
		return err;

	err = queued_retval;
	queued_retval = ERROR_OK;
	LOG_DEBUG_IO("SWD queue return value: %02x", err);

	return err;
}

static int xlnx_axi_xvc_swd_init(void)
{
	return ERROR_OK;
}

static const struct swd_driver xlnx_axi_xvc_swd_ops = {
	.init = xlnx_axi_xvc_swd_init,
	.switch_seq = xlnx_axi_xvc_swd_switch_seq,
	.read_reg = xlnx_axi_xvc_swd_read_reg,
	.write_reg = xlnx_axi_xvc_swd_write_reg,
	.run = xlnx_axi_xvc_swd_run_queue,
};

static const char * const xlnx_axi_xvc_transports[] = { "jtag", "swd", NULL };

struct adapter_driver xlnx_axi_xvc_adapter_driver = {
	.name = "xlnx_axi_xvc",
	.transports = xlnx_axi_xvc_transports,
	.commands = xlnx_axi_xvc_command_handlers,

	.init = &xlnx_axi_xvc_init,
	.quit = &xlnx_axi_xvc_quit,
	/* .speed = &xlnx_axi_xvc_speed, */
	/* .khz = &xlnx_axi_xvc_khz, */
	/* .speed_div = &xlnx_axi_xvc_div, */

	.jtag_ops = &xlnx_axi_xvc_jtag_ops,
	.swd_ops  = &xlnx_axi_xvc_swd_ops,
};
