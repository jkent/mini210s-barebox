/*
 * See file CREDITS for list of people who contributed to this
 * project.
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
 *
 */

#include <common.h>
#include <driver.h>
#include <init.h>
#include <clock.h>
#include <io.h>
#include <mci.h>
#include <sdhci.h>

#define sdhci_writel(host, val, reg)	writel(val, host->base + reg)
#define sdhci_writew(host, val, reg)	writew(val, host->base + reg)
#define sdhci_writeb(host, val, reg)	writeb(val, host->base + reg)
#define sdhci_readl(host, reg)		readl(host->base + reg)
#define sdhci_readw(host, reg)		readw(host->base + reg)
#define sdhci_readb(host, reg)		readb(host->base + reg)

struct sdhci_host {
	struct mci_host		mci;
	void __iomem		*base;
	u16 version;
};

#define to_sdhci(mci)	container_of(mci, struct sdhci_host, mci)

static void sdhci_reset(struct sdhci_host *host, u8 mask)
{
	uint64_t start;

	sdhci_writeb(host, mask, SDHCI_SOFTWARE_RESET);
	start = get_time_ns();
	while (sdhci_readb(host, SDHCI_SOFTWARE_RESET) & mask) {
		if (is_timeout(start, 100 * MSECOND)) {
			printf("MMC/SD: Host reset timeout.\n");
			return;
		}
	}
}

static void sdhci_cmd_done(struct sdhci_host *host, struct mci_cmd *cmd)
{
	int i;
	if (cmd->resp_type & MMC_RSP_136) {
		/* CRC is stripped so we need to do some shifting. */
		for (i = 0; i < 4; i++) {
			cmd->response[i] = sdhci_readl(host,
					SDHCI_RESPONSE + (3-i)*4) << 8;
			if (i != 3)
				cmd->response[i] |= sdhci_readb(host,
						SDHCI_RESPONSE + (3-i)*4-1);
		}
	} else {
		cmd->response[0] = sdhci_readl(host, SDHCI_RESPONSE);
	}
}

static void sdhci_transfer_pio(struct sdhci_host *host, struct mci_data *data)
{
	int i;
	char *offs;
	for (i = 0; i < data->blocksize; i += 4) {
		offs = data->dest + i;
		if (data->flags == MMC_DATA_READ)
			*(u32 *)offs = sdhci_readl(host, SDHCI_BUFFER);
		else
			sdhci_writel(host, *(u32 *)offs, SDHCI_BUFFER);
	}
}

static int sdhci_transfer_data(struct sdhci_host *host, struct mci_data *data,
		unsigned int start_addr)
{
	unsigned int stat, rdy, mask, timeout, block = 0;

	timeout = 1000000;
	rdy = SDHCI_INT_SPACE_AVAIL | SDHCI_INT_DATA_AVAIL;
	mask = SDHCI_DATA_AVAILABLE | SDHCI_SPACE_AVAILABLE;
	do {
		stat = sdhci_readl(host, SDHCI_INT_STATUS);
		if (stat & SDHCI_INT_ERROR) {
			printf("Error detected in status(0x%X)!\n", stat);
			return -1;
		}
		if (stat & rdy) {
			if (!(sdhci_readl(host, SDHCI_PRESENT_STATE) & mask))
				continue;
			sdhci_writel(host, rdy, SDHCI_INT_STATUS);
			sdhci_transfer_pio(host, data);
			data->dest += data->blocksize;
			if (++block >= data->blocks)
				break;
		}
		if (timeout-- > 0)
			udelay(10);
		else {
			printf("Transfer data timeout\n");
			return -1;
		}
	} while (!(stat & SDHCI_INT_DATA_END));
	return 0;
}

static int sdhci_send_command(struct mci_host *mci, struct mci_cmd *cmd,
		struct mci_data *data)
{
	struct sdhci_host *host = to_sdhci(mci);
	struct sdhci_platform_data *pdata = mci->hw_dev->platform_data;
	unsigned int stat = 0;
	int ret = 0;
	int trans_bytes = 0;
	u32 mask, flags, mode;
	unsigned int start_addr = 0;
	unsigned int retry = 10000;
	uint64_t start;

	sdhci_writel(host, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);
	mask = SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT;

	/* We shouldn't wait for data inihibit for stop commands, even
	   though they might use busy signaling */
	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		mask &= ~SDHCI_DATA_INHIBIT;

	start = get_time_ns();
	while (sdhci_readl(host, SDHCI_PRESENT_STATE) & mask) {
		if (is_timeout(start, 10 * MSECOND)) {
			printf("MMC/SD: Controller never released inhibit bit(s).\n");
			return -EIO;
		}
	}

	mask = SDHCI_INT_RESPONSE;
	if (!(cmd->resp_type & MMC_RSP_PRESENT))
		flags = SDHCI_CMD_RESP_NONE;
	else if (cmd->resp_type & MMC_RSP_136)
		flags = SDHCI_CMD_RESP_LONG;
	else if (cmd->resp_type & MMC_RSP_BUSY) {
		flags = SDHCI_CMD_RESP_SHORT_BUSY;
		mask |= SDHCI_INT_DATA_END;
	} else
		flags = SDHCI_CMD_RESP_SHORT;

	if (cmd->resp_type & MMC_RSP_CRC)
		flags |= SDHCI_CMD_CRC;
	if (cmd->resp_type & MMC_RSP_OPCODE)
		flags |= SDHCI_CMD_INDEX;
	if (data)
		flags |= SDHCI_CMD_DATA;

	/*Set Transfer mode regarding to data flag*/
	if (data != 0) {
		sdhci_writeb(host, 0xe, SDHCI_TIMEOUT_CONTROL);
		mode = SDHCI_TRNS_BLK_CNT_EN;
		trans_bytes = data->blocks * data->blocksize;
		if (data->blocks > 1)
			mode |= SDHCI_TRNS_MULTI;

		if (data->flags == MMC_DATA_READ)
			mode |= SDHCI_TRNS_READ;

		sdhci_writew(host, SDHCI_MAKE_BLKSZ(SDHCI_DEFAULT_BOUNDARY_ARG,
				data->blocksize),
				SDHCI_BLOCK_SIZE);
		sdhci_writew(host, data->blocks, SDHCI_BLOCK_COUNT);
		sdhci_writew(host, mode, SDHCI_TRANSFER_MODE);
	}

	sdhci_writel(host, cmd->cmdarg, SDHCI_ARGUMENT);
	sdhci_writew(host, SDHCI_MAKE_CMD(cmd->cmdidx, flags), SDHCI_COMMAND);
	do {
		stat = sdhci_readl(host, SDHCI_INT_STATUS);
		if (stat & SDHCI_INT_ERROR)
			break;
		if (--retry == 0)
			break;
	} while ((stat & mask) != mask);

	if (retry == 0) {
		if (pdata && pdata->quirks & SDHCI_QUIRK_BROKEN_R1B)
			return 0;
		else {
			printf("Timeout for status update!\n");
			return -ETIMEDOUT;
		}
	}

	if ((stat & (SDHCI_INT_ERROR | mask)) == mask) {
		sdhci_cmd_done(host, cmd);
		sdhci_writel(host, mask, SDHCI_INT_STATUS);
	} else
		ret = -1;

	if (!ret && data)
		ret = sdhci_transfer_data(host, data, start_addr);

	if (pdata && pdata->quirks & SDHCI_QUIRK_WAIT_SEND_CMD)
		udelay(1000);

	stat = sdhci_readl(host, SDHCI_INT_STATUS);
	sdhci_writel(host, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);
	if (!ret)
		return 0;

	sdhci_reset(host, SDHCI_RESET_CMD);
	sdhci_reset(host, SDHCI_RESET_DATA);
	if (stat & SDHCI_INT_TIMEOUT)
		return -ETIMEDOUT;
	else
		return -EIO;
}

static int sdhci_set_clock(struct mci_host *mci, unsigned int clock)
{
	struct sdhci_host *host = to_sdhci(mci);
	unsigned int div, clk;
	uint64_t start;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		return 0;

	if ((host->version & SDHCI_SPEC_VER_MASK) >= SDHCI_SPEC_300) {
		if (mci->f_max <= clock)
			div = 1;
		else {
			for (div = 2; div < SDHCI_MAX_DIV_SPEC_300; div += 2) {
				if ((mci->f_max / div) <= clock)
					break;
			}
		}
	} else {
		for (div = 1; div < SDHCI_MAX_DIV_SPEC_200; div *= 2) {
			if ((mci->f_max / div) <= clock)
				break;
		}
	}
	div >>= 1;

	//if (host->set_clock)
	//	host->set_clock(host->index, div);

	clk = (div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
	clk |= ((div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN)
		<< SDHCI_DIVIDER_HI_SHIFT;
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	start = get_time_ns();
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (is_timeout(start, 20 * MSECOND)) {
			printf("MMC/SD: Internal clock never stabilised.\n");
			return -1;
		}
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
	return 0;
}

static void sdhci_set_power(struct sdhci_host *host, u16 power)
{
	struct sdhci_platform_data *pdata = host->mci.hw_dev->platform_data;
	u8 pwr = 0;

	if (power != (unsigned short)-1) {
		switch (1 << power) {
		case MMC_VDD_165_195:
			pwr = SDHCI_POWER_180;
			break;
		case MMC_VDD_29_30:
		case MMC_VDD_30_31:
			pwr = SDHCI_POWER_300;
		case MMC_VDD_32_33:
		case MMC_VDD_33_34:
			pwr = SDHCI_POWER_330;
			break;
		}
	}

	if (pwr == 0) {
		sdhci_writeb(host, 0, SDHCI_POWER_CONTROL);
		return;
	}

	if (pdata && pdata->quirks & SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER)
		sdhci_writeb(host, pwr, SDHCI_POWER_CONTROL);

	pwr |= SDHCI_POWER_ON;

	sdhci_writeb(host, pwr, SDHCI_POWER_CONTROL);
}

#ifdef CONFIG_ARCH_S5PCxx

#define SDHCI_CONTROL2		0x80
#define SDHCI_CONTROL3		0x84
#define SDHCI_CONTROL4		0x8C

#define SDHCI_CTRL2_ENSTAASYNCCLR	(1 << 31)
#define SDHCI_CTRL2_ENCMDCNFMSK		(1 << 30)
#define SDHCI_CTRL2_CDINVRXD3		(1 << 29)
#define SDHCI_CTRL2_SLCARDOUT		(1 << 28)

#define SDHCI_CTRL2_FLTCLKSEL_MASK	(0xf << 24)
#define SDHCI_CTRL2_FLTCLKSEL_SHIFT	(24)
#define SDHCI_CTRL2_FLTCLKSEL(_x)	((_x) << 24)

#define SDHCI_CTRL2_LVLDAT_MASK		(0xff << 16)
#define SDHCI_CTRL2_LVLDAT_SHIFT	(16)
#define SDHCI_CTRL2_LVLDAT(_x)		((_x) << 16)

#define SDHCI_CTRL2_ENFBCLKTX		(1 << 15)
#define SDHCI_CTRL2_ENFBCLKRX		(1 << 14)
#define SDHCI_CTRL2_SDCDSEL		(1 << 13)
#define SDHCI_CTRL2_SDSIGPC		(1 << 12)
#define SDHCI_CTRL2_ENBUSYCHKTXSTART	(1 << 11)

#define SDHCI_CTRL2_DFCNT_MASK(_x)	((_x) << 9)
#define SDHCI_CTRL2_DFCNT_SHIFT		(9)

#define SDHCI_CTRL2_ENCLKOUTHOLD	(1 << 8)
#define SDHCI_CTRL2_RWAITMODE		(1 << 7)
#define SDHCI_CTRL2_DISBUFRD		(1 << 6)
#define SDHCI_CTRL2_SELBASECLK_MASK(_x)	((_x) << 4)
#define SDHCI_CTRL2_SELBASECLK_SHIFT	(4)
#define SDHCI_CTRL2_PWRSYNC		(1 << 3)
#define SDHCI_CTRL2_ENCLKOUTMSKCON	(1 << 1)
#define SDHCI_CTRL2_HWINITFIN		(1 << 0)

#define SDHCI_CTRL3_FCSEL3		(1 << 31)
#define SDHCI_CTRL3_FCSEL2		(1 << 23)
#define SDHCI_CTRL3_FCSEL1		(1 << 15)
#define SDHCI_CTRL3_FCSEL0		(1 << 7)

#define SDHCI_CTRL4_DRIVE_MASK(_x)	((_x) << 16)
#define SDHCI_CTRL4_DRIVE_SHIFT		(16)

static void s5p_sdhci_set_control_reg(struct sdhci_host *host)
{
	unsigned long val, ctrl;
	/*
	 * SELCLKPADDS[17:16]
	 * 00 = 2mA
	 * 01 = 4mA
	 * 10 = 7mA
	 * 11 = 9mA
	 */
	sdhci_writel(host, SDHCI_CTRL4_DRIVE_MASK(0x3), SDHCI_CONTROL4);

	val = sdhci_readl(host, SDHCI_CONTROL2);
	val &= SDHCI_CTRL2_SELBASECLK_SHIFT;

	val |=	SDHCI_CTRL2_ENSTAASYNCCLR |
		SDHCI_CTRL2_ENCMDCNFMSK |
		SDHCI_CTRL2_ENFBCLKRX |
		SDHCI_CTRL2_ENCLKOUTHOLD;

	sdhci_writel(host, val, SDHCI_CONTROL2);

	/*
	 * FCSEL3[31] FCSEL2[23] FCSEL1[15] FCSEL0[7]
	 * FCSel[1:0] : Rx Feedback Clock Delay Control
	 *	Inverter delay means10ns delay if SDCLK 50MHz setting
	 *	01 = Delay1 (basic delay)
	 *	11 = Delay2 (basic delay + 2ns)
	 *	00 = Delay3 (inverter delay)
	 *	10 = Delay4 (inverter delay + 2ns)
	 */
	val = SDHCI_CTRL3_FCSEL0 | SDHCI_CTRL3_FCSEL1;
	sdhci_writel(host, val, SDHCI_CONTROL3);

	/*
	 * SELBASECLK[5:4]
	 * 00/01 = HCLK
	 * 10 = EPLL
	 * 11 = XTI or XEXTCLK
	 */
	ctrl = sdhci_readl(host, SDHCI_CONTROL2);
	ctrl &= ~SDHCI_CTRL2_SELBASECLK_MASK(0x3);
	ctrl |= SDHCI_CTRL2_SELBASECLK_MASK(0x2);
	sdhci_writel(host, ctrl, SDHCI_CONTROL2);
}
#endif

static void sdhci_set_ios(struct mci_host *mci, struct mci_ios *ios)
{
	struct sdhci_host *host = to_sdhci(mci);
	struct sdhci_platform_data *pdata = mci->hw_dev->platform_data;
	u8 ctrl;

//#ifdef CONFIG_ARCH_S5PCxx
	s5p_sdhci_set_control_reg(host);
//#endif

	sdhci_set_clock(mci, ios->clock);

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);

	if (ios->bus_width == 8) {
		ctrl &= ~SDHCI_CTRL_4BITBUS;
		if ((host->version & SDHCI_SPEC_VER_MASK) >= SDHCI_SPEC_300)
			ctrl |= SDHCI_CTRL_8BITBUS;
	} else {
		if ((host->version & SDHCI_SPEC_VER_MASK) >= SDHCI_SPEC_300)
			ctrl &= ~SDHCI_CTRL_8BITBUS;
		if (ios->bus_width == 4)
			ctrl |= SDHCI_CTRL_4BITBUS;
		else
			ctrl &= ~SDHCI_CTRL_4BITBUS;
	}

	if (ios->clock > 26000000)
		ctrl |= SDHCI_CTRL_HISPD;
	else
		ctrl &= ~SDHCI_CTRL_HISPD;

	if (pdata && pdata->quirks & SDHCI_QUIRK_NO_HISPD_BIT)
		ctrl &= ~SDHCI_CTRL_HISPD;

	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

static int sdhci_card_present(struct mci_host *mci)
{
	struct sdhci_host *host = to_sdhci(mci);
	struct sdhci_platform_data *pdata = mci->hw_dev->platform_data;

	if (pdata && pdata->quirks & SDHCI_QUIRK_NO_CD)
		return 1;

	return !!(sdhci_readl(host, SDHCI_PRESENT_STATE) & SDHCI_CARD_PRESENT);
}

static int sdhci_init(struct mci_host *mci, struct device_d *dev)
{
	struct sdhci_host *host = to_sdhci(mci);
	struct sdhci_platform_data *pdata = mci->hw_dev->platform_data;

	sdhci_set_power(host, fls(mci->voltages) - 1);

	if (pdata->quirks & SDHCI_QUIRK_NO_CD) {
		unsigned int status;

		sdhci_writel(host, SDHCI_CTRL_CD_TEST_INS | SDHCI_CTRL_CD_TEST,
			SDHCI_HOST_CONTROL);

		status = sdhci_readl(host, SDHCI_PRESENT_STATE);
		while ((!(status & SDHCI_CARD_PRESENT)) ||
			(!(status & SDHCI_CARD_STATE_STABLE)) ||
			(!(status & SDHCI_CARD_DETECT_PIN_LEVEL)))
				status = sdhci_readl(host, SDHCI_PRESENT_STATE);
	}

	sdhci_writel(host, SDHCI_INT_DATA_MASK | SDHCI_INT_CMD_MASK,
			SDHCI_INT_ENABLE);

	sdhci_writel(host, 0, SDHCI_SIGNAL_ENABLE);

	return 0;
}

static int sdhci_probe(struct device_d *dev)
{
	struct sdhci_host *host;
	struct mci_host *mci;
	struct sdhci_platform_data *pdata = dev->platform_data;
	u32 caps;

	host = xzalloc(sizeof(*host));
	mci = &host->mci;

	host->base = dev_request_mem_region(dev, 0);

	mci->send_cmd = sdhci_send_command;
	mci->set_ios = sdhci_set_ios;
	mci->init = sdhci_init;
	mci->card_present = sdhci_card_present;
	mci->hw_dev = dev;

	host->version = sdhci_readw(host, SDHCI_HOST_VERSION);

	caps = sdhci_readl(host, SDHCI_CAPABILITIES);

	if (pdata && pdata->max_clk) {
		mci->f_max = pdata->max_clk;
	} else {
		if ((host->version & SDHCI_SPEC_VER_MASK) >= SDHCI_SPEC_300)
			mci->f_max = (caps & SDHCI_CLOCK_V3_BASE_MASK)
				>> SDHCI_CLOCK_BASE_SHIFT;
		else
			mci->f_max = (caps & SDHCI_CLOCK_BASE_MASK)
				>> SDHCI_CLOCK_BASE_SHIFT;
		mci->f_max *= 1000000;
	}
	if (mci->f_max == 0) {
		printf("MMC/SD: Hardware doesn't specify base clock" \
				" frequency\n");
		return -1;
	}
	if (pdata && pdata->min_clk) {
		mci->f_min = pdata->min_clk;
	} else {
		if ((host->version & SDHCI_SPEC_VER_MASK) >= SDHCI_SPEC_300)
			mci->f_min = mci->f_max / SDHCI_MAX_DIV_SPEC_300;
		else
			mci->f_min = mci->f_max / SDHCI_MAX_DIV_SPEC_200;
	}

	if (caps & SDHCI_CAN_VDD_330)
		mci->voltages |= MMC_VDD_32_33 | MMC_VDD_33_34;
	if (caps & SDHCI_CAN_VDD_300)
		mci->voltages |= MMC_VDD_29_30 | MMC_VDD_30_31;
	if (caps & SDHCI_CAN_VDD_180)
		mci->voltages |= MMC_VDD_165_195;

	if (pdata && pdata->quirks & SDHCI_QUIRK_BROKEN_VOLTAGE)
		mci->voltages |= pdata->voltages;

	mci->host_caps = MMC_MODE_HS | MMC_MODE_HS_52MHz | MMC_MODE_4BIT;
	/* Shouldn't advertise 8-bit support unless the physical
	 * implementation actually supports it.
	 */
	/*
	if (caps & SDHCI_CAN_DO_8BIT)
		mci->host_caps |= MMC_MODE_8BIT;
	*/
	if (pdata && pdata->caps)
		mci->host_caps |= pdata->caps;

	sdhci_reset(host, SDHCI_RESET_ALL);
	mci_register(&host->mci);

	return 0;
}

static struct driver_d sdhci_driver = {
	.name  = "sdhci",
	.probe = sdhci_probe,
};
device_platform_driver(sdhci_driver);

struct device_d *add_sdhci_device(int id, resource_size_t base, void *pdata)
{
	return add_generic_device("sdhci", id, NULL, base, 0x100,
			IORESOURCE_MEM, pdata);
}
