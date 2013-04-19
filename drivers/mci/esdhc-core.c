/*
 * Copyright (C) 2012 Juergen Beisert
 *
 * This code is partially based on code from:
 *
 *  the FriendlyARM developers Jinsung Yang and Jongpill Lee. They did not add
 *  their copyright note, so I extracted their names from a logfile.
 *
 *  Copyright 2000-2006 Samsung Electronics sc.suh@samsung.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

// #define DEBUG

#include <common.h>
#include <init.h>
#include <clock.h>
#include <mci.h>
#include <mci/esdhc.h>
#include <errno.h>
#include <clock.h>
#include <io.h>

#include "esdhc-core.h"

#define to_esdhc(mci) container_of(mci, struct esdhc_mci, mci)

/* the platform driver must support the following API */

/*
 * initialize the ESDHC. Should return:
 *  0: successfull
 *  everything else on failure
 */
int esdhcp_initialize(struct esdhc_mci *h);

/*
 * check if an SD/MMC card is plugged in. Should return:
 *  0: if a card is present
 *  -ENOSYS: if it can't detect the status
 *  -ENODEV: if no card is present
 */
int esdhcp_card_present(struct esdhc_mci *);

/*
 * check if the SD/MMC card is write protected. Should return:
 *  0: if the card is write enabled
 *  -ENOSYS: if it can't detect the status
 *  -EROFS: if the card is write protected
 */
int esdhcp_card_protected(struct esdhc_mci *);

/*
 * setup the physical features of the ESDHC's interface. Should return:
 *  0: successfull
 *  everything else on failure
 */
int esdhcp_interface_setup(struct esdhc_mci *, unsigned);

/*
 * setup a new clock frequency use to communicate with the SD/MMC card.
 * Should return the physically used frequency:
 *  0: if the new clock frequency cannot be set
 */
unsigned esdhcp_setup_clock(struct esdhc_mci *, unsigned);

/* ------------------------ SDA generic part ------------------------------ */

/* enable the clock to the _external_ SD/MMC card */
void esdhc_enable_SD_clock(struct esdhc_mci *h)
{
	uint16_t reg = readw(h->base + CLKCON);
	writew(reg | CLKCON_ENSDCLK, h->base + CLKCON);
	pr_debug("SD Clock enabled\n");
	/* TODO check bit 4: it will be set, when the SD clock is active */
	/* note: after setting bit 3 it needs some time to activate the SD clock! But it seems very fast */
}

/* disable the clock to the _external_ SD/MMC card */
void esdhc_disable_SD_clock(struct esdhc_mci *h)
{
	uint16_t reg = readw(h->base + CLKCON) & ~CLKCON_ENSDCLK;
	writew(reg, h->base + CLKCON);
	pr_debug("SD Clock disabled\n");
	/* TODO check bit 4: it will be cleared, when the SD clock is inactive */
	/* note: after clearing bit 3 it needs some time to disable the SD clock! It seems very slow! 1 second */
}

/* setup data transfer related register */
static int esdhc_prepare_data_transfer(struct esdhc_mci *h, struct mci_data *data)
{
	u16 trnmode = 0;

	writeb(0x0e, h->base + TIMEOUTCON); /* TODO 2^27? */

	if (data->blocks > 1)
		trnmode |= TRNMOD_MUL1SIN0 | TRNMOD_ENBLKCNT;
	if (data->flags == MMC_DATA_READ)
		trnmode |= TRNMOD_RD1WT0;

	writew(data->blocksize, h->base + BLKSIZE);
	writew(data->blocks, h->base + BLKCNT);
	writeb(0, h->base + BLKGAP);

	writew(trnmode, h->base + TRNMOD);
	pr_debug("%s: %d blocks of size 0x200, mode %04hX\n", __func__, data->blocks, trnmode);

	return 0;
}

static int esdhc_terminate_transfer(struct esdhc_mci *h)
{
	return -EINVAL;
}

static unsigned esdhc_get_cmd_flags(unsigned cmd_flags, unsigned data_flags)
{
	unsigned reg = CMDREG_RSPTYP_NONE;

	if (cmd_flags & MMC_RSP_136) {
		reg |= CMDREG_RSPTYP_136B;
		pr_debug("Command with long response\n");
	} else if (cmd_flags & MMC_RSP_BUSY) {
		reg |= CMDREG_RSPTYP_48B_BUSY;
		pr_debug("Command with response & busy\n");
	} else if (cmd_flags & MMC_RSP_PRESENT) {
		reg |= CMDREG_RSPTYP_48B;
		pr_debug("Command with response\n");
	}

	if (cmd_flags & MMC_RSP_CRC) {
		reg |= CMDREG_ENCMDCRC;
		pr_debug("Command with CRC response\n");
	}

	if (cmd_flags & MMC_RSP_OPCODE) {
		reg |= CMDREG_ENCMDIDC;
		pr_debug("Command with OPCODE(FIXME)\n");
	}

	if (data_flags != 0) {
		reg |= CMDREG_DATAPRNT;
		pr_debug("Command with data\n");
	}

	return reg;
}

/* wait until the card accepts the next command */
static int esdhc_wait_for_command_ready(struct esdhc_mci *h, u32 mask)
{
	int rc = wait_on_timeout(1 * SECOND, !(readl(h->base + PRNSTS) & mask));
	if (rc != 0)
		pr_debug("Waiting for SD card timed out -> %08X\n", readl(h->base + PRNSTS));
	return rc;
}

/* wait until the current commend is done */
static int esdhc_cmd_wait_done(struct esdhc_mci *h, unsigned mask)
{
	u16 reg, err;
	uint64_t start;
	static const uint64_t toffs = 1 * SECOND;

	/* enable the status bit reports */
	writew(NORINTSTS_STACMDCMPLT | mask, h->base + NORINTSTSEN);

	pr_debug("%s: waiting for bit pattern %X\n", __func__, NORINTSTS_STACMDCMPLT | mask);

	start = get_time_ns();
	do {
		reg = readw(h->base + NORINTSTS);
		if (reg & NORINTSTS_STAERR) {
			pr_debug("Controller reports an error\n");
			break;
		}
		if ((reg & (NORINTSTS_STACMDCMPLT | mask)) == (NORINTSTS_STACMDCMPLT | mask)) {
			pr_debug("Command complete\n");
			writew(NORINTSTS_STACMDCMPLT | mask, h->base + NORINTSTS);
			return 0;
		}
	} while (!is_timeout(start, toffs));
	if (is_timeout(start, toffs))
		pr_debug("Command timed out!\n");

	err = readw(h->base + ERRINTSTS);
	/* clear the error flags */
	writew(err, h->base + ERRINTSTS);
	writew(reg, h->base + NORINTSTS);
	pr_debug("%s command error: 0x%04hx -> 0x%04hx\n", __func__, reg, err);
	return -ETIMEDOUT;
}

static int esdhc_transfer_wait_done(struct esdhc_mci *h)
{
	unsigned status = readl(h->base + PRNSTS);

	if (status & (PRNSTS_RDTRANACT | PRNSTS_WTTRANACT))
		pr_warn("Transaction still in progress\n"); /* TODO */

	return 0;
}

static void esdhc_shift_respond(u32 *resp, unsigned *answ)
{
	answ[0] = (resp[3] << 8) | (resp[2] >> 24);
	answ[1] = (resp[2] << 8) | (resp[1] >> 24);
	answ[2] = (resp[1] << 8) | (resp[0] >> 24);
	answ[3] = (resp[0] << 8);
}

/* issue a command, wait untils it's done and read the response */
static int esdhc_cmd_send(struct esdhc_mci *h, struct mci_cmd *cmd, struct mci_data *data)
{
	int rc;
	u32 mask = PRNSTS_CMDINHCMD, resp[4];
	u16 reg;

	pr_debug("  %s called\n", __func__);

	if (cmd->cmdidx != MMC_CMD_STOP_TRANSMISSION)
		mask |= PRNSTS_CMDINHDAT;

	rc = esdhc_wait_for_command_ready(h, mask);
	if (rc != 0)
		return rc;

	reg = esdhc_get_cmd_flags(cmd->resp_type, data != NULL ? data->flags : 0);

	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		reg |= CMDREG_CMDTYP_IOABRT;

	reg |= CMDREG_CMDIDX(cmd->cmdidx);

	pr_debug("  %s writing argument: %X\n", __func__, cmd->cmdarg);
	writel(cmd->cmdarg, h->base + ARGMNT);
	pr_debug("  %s writing command %02X: %X\n", __func__, cmd->cmdidx, reg);
	writew(reg, h->base + CMDREG); /* issue the command after all data */

	rc = esdhc_cmd_wait_done(h, /*cmd->resp_type & MMC_RSP_BUSY ? NORINTSTS_STATRANCMPLT :*/ 0);
	if (rc != 0)
		return rc;

	pr_debug("  %s reading status\n", __func__);

	cmd->response[0] = resp[0] = readl(h->base + RSPREG0);
	if (cmd->resp_type & MMC_RSP_136) {
		resp[1] = readl(h->base + RSPREG1);
		resp[2] = readl(h->base + RSPREG2);
		resp[3] = readl(h->base + RSPREG3);
		esdhc_shift_respond(resp, cmd->response);
	}

	pr_debug("  %s left successfull\n", __func__);
	return 0;
}

/* wait until the data is written from the internal FIFO to the card */
static int esdhc_wait_buffer_write_ready(struct esdhc_mci *h)
{
	writew(NORINTSTS_STABUFWTRDY, h->base + NORINTSTSEN);
	return wait_on_timeout(1 * SECOND, !(readw(h->base + NORINTSTS) & NORINTSTS_STABUFWTRDY));
// 	return wait_on_timeout(1 * SECOND, !(readl(h->base + PRNSTS) & PRNSTS_BUFWTRDY));
}

/* transfer data from host to card */
static int esdhc_write_pio(struct esdhc_mci *h, struct mci_data *data)
{
	const u32 *p;
	unsigned cnt, data_size;
	int rc;

	p = (const u32*)data->src;
	data_size = data->blocksize * data->blocks;

	while (data_size > 0) {
		/* wait for free entries in the FIFO */
		rc = esdhc_wait_buffer_write_ready(h);
		if (rc != 0)
			return rc;

		cnt = 512; /* FIXME must we always write a full block? */
		while (cnt--) {
			writel(*p++, h->base + BDATA);
			if (data_size > 4)
				data_size -= 4;
			else {
				data_size = 0;
				break;
			}
		}
	}

	return 0;
}

/* transfer data from card to host */
static int esdhc_read_pio(struct esdhc_mci *h, struct mci_data *data)
{
	struct device_d *dev = h->mci.hw_dev;
	uint64_t start;
	static const uint64_t toffs = 1 * SECOND;
	unsigned cnt, data_size;
	u32 *p;
	int rc;

	/*
	 * it is important to wait for the first data in the FIFO to arrive.
	 * And then delay a short time. Otherwise the data from the previous
	 * transfer is read again and then the unit times out. Happen with
	 * sequential reads of 8 bytes and 64 bytes. Without this pause the
	 * second read delivers the 8 bytes again and then stops.
	 *
	 * It seems this unit needs some time to fill the FIFO after it has
	 * signal the data via PRNSTS_BUFRDRDY. At least at lower SD clocks
	 * this is an issue.
	 *
	 * Issue happend on an S3C6410 SoC
	 */
	rc = wait_on_timeout(1 * SECOND,
				readl(h->base + PRNSTS) & PRNSTS_BUFRDRDY);
	if (rc) {
		dev_err(dev, "Reading card's data timed out\n");
		return -ETIMEDOUT;
	}

	p = (u32*)data->dest;
	data_size = data->blocksize * data->blocks;

	while (data_size > 0) {
		cnt = (data->blocksize + 3) / 4;
		start = get_time_ns();

		while (cnt) {
			while (readl(h->base + PRNSTS) & PRNSTS_BUFRDRDY) {
				*p++ = readl(h->base + BDATA);
				if (data_size > 4) {
					data_size -= 4;
					cnt--;
				} else {
					data_size = cnt = 0;
					break;
				}
			}

			if (is_timeout(start, toffs)) {
				pr_err("More data expected (%u, %X)\n", data_size, readl(h->base + PRNSTS));
				return -ETIMEDOUT;
			}
		}
	}

	return 0;
}

static int esdhc_data_cmds(struct esdhc_mci *h, struct mci_cmd *cmd, struct mci_data *data)
{
	int rc;

	rc = esdhc_prepare_data_transfer(h, data);
	if (rc != 0)
		return rc;

	if (data->flags & MMC_DATA_READ) {
		/* command phase */
		pr_debug("%s: read transfer, command phase\n", __func__);
		rc = esdhc_cmd_send(h, cmd, data);
		if (rc != 0)
			return rc;
		/* data phase */
		pr_debug("%s: read transfer, data phase\n", __func__);
		rc = esdhc_read_pio(h, data);
		if (rc == 0)
			/* wait until it's done */
			rc = esdhc_transfer_wait_done(h);
		else
			esdhc_terminate_transfer(h);
		pr_debug("%s: read transfer done (%d)\n", __func__, rc);
		return rc;
	}

	if (data->flags & MMC_DATA_WRITE) {
		rc = esdhcp_card_protected(h);
		if (rc != 0)
			return rc;

		/* command phase */
		pr_debug("%s: write transfer, command phase\n", __func__);
		rc = esdhc_cmd_send(h, cmd, data);
		if (rc != 0)
			return rc;

		/* data phase */
		pr_debug("%s: write transfer, data phase\n", __func__);
		rc = esdhc_write_pio(h, data);
		if (rc == 0)
			/* wait until it's done */
			rc = esdhc_transfer_wait_done(h);
		else
			esdhc_terminate_transfer(h);
		pr_debug("%s: write transfer done(%d)\n", __func__, rc);
	}

	return rc;
}

/* ------------------------- MCI API -------------------------------------- */

static int esdhc_reset(struct mci_host *mci, struct device_d *mci_dev)
{
	struct esdhc_mci *h = to_esdhc(mci);

	return esdhcp_initialize(h);
}

static int esdhc_request(struct mci_host *mci, struct mci_cmd *cmd, struct mci_data *data)
{
	struct esdhc_mci *h = to_esdhc(mci);
	int rc;

	pr_debug("%s called\n", __func__);
	esdhc_enable_SD_clock(h);

	rc = esdhcp_card_present(h);
	if (rc == 0) {
		if ((cmd->resp_type == 0) || (data == NULL)) {
			pr_debug(" %s: Send command only\n", __func__);
			writew(0x0000, h->base + TRNMOD); /* FIXME */
			rc = esdhc_cmd_send(h, cmd, NULL);
		} else {
			pr_debug(" %s: Send command with data\n", __func__);
			rc = esdhc_data_cmds(h, cmd, data);	/* with response and data */
		}
	}

	esdhc_disable_SD_clock(h);
	pr_debug("%s left\n", __func__);
	return rc;
}

/* change transfer clock speed and/or data bus width */
static void esdhc_set_ios(struct mci_host *mci, struct mci_ios *ios)
{
	struct esdhc_mci *h = to_esdhc(mci);
	u8 reg;

	pr_debug("%s called\n", __func__);
	/* data bus width */
	reg = readb(h->base + HOSTCTL) & ~(HOSTCTL_WIDE4 | HOSTCTL_WIDE8);
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		break;
	case MMC_BUS_WIDTH_4:
		reg |= HOSTCTL_WIDE4;
		break;
	case MMC_BUS_WIDTH_8:
		reg |= HOSTCTL_WIDE8;
		break;
	}
	writeb(reg, h->base + HOSTCTL);
	h->bus_width = ios->bus_width;

	if (ios->clock == 0)
		pr_debug("Should switch off the clock\n");
	else
		h->clock = esdhcp_setup_clock(h, ios->clock);
	pr_debug("%s left\n", __func__);
}

/* ----------------------------------------------------------------------- */

#ifdef CONFIG_MCI_INFO
static void esdhc_print_ver(struct esdhc_mci *h)
{
	u16 reg;

	reg = readw(h->base + HCVER);
	printf("ESDHC %u.0 HC, spec %u.0\n", HCVER_GET_VENVER(reg), HCVER_GET_SPECVER(reg));
}

static void esdhc_print_caps(struct esdhc_mci *h)
{
	u32 reg;

	reg = readl(h->base + CAPAREG);
	printf("  Bus voltages:\n");
	if (reg & CAPAREG_CAPV18)
		printf("   1.8V\n");
	if (reg & CAPAREG_CAPV30)
		printf("   3.0V\n");
	if (reg & CAPAREG_CAPV33)
		printf("   3.3V\n");
	printf("  DMA capability: %s\n", reg & CAPAREG_CAPDMA ? "yes" : "no");
	printf("  High speed capability: %s\n", reg & CAPAREG_CAPAHSPD ? "yes" : "no");
	printf("  Max block length: ");
	switch (CAPAREG_GET_CAPMAXBLKLEN(reg)) {
	case 0:
		printf("512 bytes\n");
		break;
	case 1:
		printf("1 kiBytes\n");
		break;
	case 2:
		printf("2 kiBytes\n");
		break;
	default:
		printf("(unknown)\n");
		break;
	}
}

void esdhc_info(struct device_d *dev)
{
	struct esdhc_mci *h = dev->priv;
	struct esdhc_platform_data *pd = h->pdata;

	esdhc_print_ver(h);
	esdhc_print_caps(h);

	printf("  Bus data width: %d bit\n", h->bus_width == 1 ? 4 : 1);
	printf("  Bus frequency: %u Hz\n", h->clock);
	printf("   Frequency limits: ");

	if (pd->f_min == 0)
		printf("no lower limit ");
	else
		printf("%u Hz lower limit ", pd->f_min);
	if (pd->f_max == 0)
		printf("- no upper limit");
	else
		printf("- %u Hz upper limit", pd->f_max);

	printf("\n  Card detection support: %s\n",
		pd->cd_type == ESDHC_CD_NONE ||
		pd->cd_type == ESDHC_CD_PERMANENT ? "no" : "yes");

	printf("  Card write protection support: %s\n",
		pd->wp_type == ESDHC_WP_NONE ||
		pd->wp_type == ESDHC_WP_CONTROLLER ? "no" : "yes");
}
#endif

int esdhc_register(struct esdhc_mci *h)
{
	h->mci.send_cmd = esdhc_request;
	h->mci.set_ios = esdhc_set_ios;
	h->mci.init = esdhc_reset;

	/*
	 * Start the clock to let the engine and the card finishes its startup
	 */
	h->clock = esdhcp_setup_clock(h, h->mci.f_min);

	return mci_register(&h->mci);
}
