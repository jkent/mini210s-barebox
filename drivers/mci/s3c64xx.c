/*
 * Copyright (C) 2012 Juergen Beisert
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

#define DEBUG

#include <common.h>
#include <init.h>
#include <io.h>
#include <clock.h>
#include <errno.h>
#include <gpio.h>
#include <mci.h>
#include <mci/esdhc.h>
#include <mach/s3c-esdhc.h>
#include <mach/s3c-generic.h>
#include <mach/s3c-iomap.h>

#include "esdhc-core.h"

/* implementation defined SDHC registers */
#define CONTROL2 0x80	/* u32 */
# define CONTROL2_ENSTAASYNCCLR (1 << 31)
# define CONTROL2_ENCMDCNFMSK (1 << 30)
# define CONTROL2_CDINVRXD3 (1 << 29)
# define CONTROL2_ENFBCLKTX (1 << 15)
# define CONTROL2_ENFBCLKRX (1 << 14)
# define CONTROL2_DEFCNT(x) ((x) << 9)
# define CONTROL2_ENCLKOUTHOLD (1 << 8)
# define CONTROL2_SELBASECLK(x) ((x) << 4)
# define CONTROL2_ENCLKOUTMSKCON (1 << 1)
# define CONTROL2_HWINITFIN (1 << 0)
#define CONTROL3 0x84	/* u32 */
# define CONTROL3_FCSEL3 (1 << 31)
# define CONTROL3_FIA3(x) ((x) << 24)
# define CONTROL3_FCSEL2 (1 << 23)
# define CONTROL3_FIA2(x) ((x) << 16)
# define CONTROL3_FCSEL1 (1 << 15)
# define CONTROL3_FIA1(x) ((x) << 8)
# define CONTROL3_FCSEL0 (1 << 7)
# define CONTROL3_FIA0(x) (x)
# define CONTROL3_FCSEL_MASK (CONTROL3_FCSEL0 | CONTROL3_FCSEL1 | CONTROL3_FCSEL2 | CONTROL3_FCSEL3)
#define CONTROL4 0x8C	/* u32 */
# define CONTROL4_SELCLKPADDS(x) ((x) << 16)
# define CONTROL4_STABUSY (1 << 0)

#define to_s3c_pdata(pdata) \
	container_of(pdata, struct s3c_sdhc_platform_data, esdhc_pd)

/* return 0=card present, else: error code */
int esdhcp_card_present(struct esdhc_mci *h)
{
	int rc;

	switch (h->pdata->cd_type) {
	case ESDHC_CD_NONE:
		return -ENOSYS;	/* TODO */

	case ESDHC_CD_PERMANENT:
		/* assume its present */
		break;

	case ESDHC_CD_CONTROLLER:
		rc = wait_on_timeout(1 * SECOND,
				readl(h->base + PRNSTS) & PRNSTS_STBLCARD);
		if (rc != 0)
			return rc;
		if (!(readl(h->base + PRNSTS) & PRNSTS_INSCARD))
			return -ENODEV;
		break;

	case ESDHC_CD_GPIO:
		rc = gpio_direction_input(h->pdata->cd_gpio);
		if (rc != 0)
			return rc;
		if (gpio_get_value(h->pdata->cd_gpio))
			return -ENODEV;
		break;

	case ESDHC_CD_GPIO_INV:
		rc = gpio_direction_input(h->pdata->cd_gpio);
		if (rc != 0)
			return rc;
		if (!gpio_get_value(h->pdata->cd_gpio))
			return -ENODEV;
		break;
	}

	return 0;
}

int esdhcp_card_protected(struct esdhc_mci *h)
{
	int rc;

	switch (h->pdata->wp_type) {
	case ESDHC_WP_CONTROLLER:
	case ESDHC_WP_NONE:
		return -ENOSYS;	/* TODO */

	case ESDHC_WP_PERMANENT:
		break;

	case ESDHC_WP_GPIO:
		rc = gpio_direction_input(h->pdata->wp_gpio);
		if (rc != 0)
			return rc;
		if (!gpio_get_value(h->pdata->wp_gpio))
			return -EROFS;
		break;

	case ESDHC_WP_GPIO_INV:
		rc = gpio_direction_input(h->pdata->wp_gpio);
		if (rc != 0)
			return rc;
		if (gpio_get_value(h->pdata->wp_gpio))
			return -EROFS;
		break;
	}

	return 0;
}

/*
 * Clock feedback control:
 *
 * Rx: FCSEL0 + FCSEL1
 *  - 00 inverter delay (=10ns @ 50 MHz, FIXME and @ <50 MHz?)
 *  - 01 basic delay
 *  - 10 inverter delay + 2ns (=12ns @ 50 MHz)
 *  - 11 basic delay + 2ns
 *
 * clk source --------o--------------------------------o--------> SD/MMC card
 *                    |                                |
 * feedback <---------X----------O---------------------o
 *                    ^          ^                     |
 *                ENFBCLKTX      O-- 2ns -O------------o
 *                               |        ^            |
 *                               +--------O- inverter -+
 */
static unsigned esdhc_tx_delay_set(struct esdhc_mci *h, unsigned val)
{
	u32 reg = 0;

	if (val & 0x01)
		reg |= CONTROL3_FCSEL2;
	if (val & 0x02)
		reg |= CONTROL3_FCSEL3;

	return reg;
}

/*
 * Clock feedback control:
 *
 * Rx: FCSEL0 + FCSEL1
 *  - 00 inverter delay (=10ns @ 50 MHz, FIXME and @ <50 MHz?)
 *  - 01 basic delay
 *  - 10 inverter delay + 2ns (=12ns @ 50 MHz)
 *  - 11 basic delay + 2ns
 *
 * clk source --------o--------------------------------o--------> SD/MMC card
 *                    |                                |
 * feedback <---------X----------O---------------------o
 *                    ^          ^                     |
 *                ENFBCLKRX      O-- 2ns -O------------o
 *                               |        ^            |
 *                               +--------O- inverter -+
 */
static unsigned esdhc_rx_delay_set(struct esdhc_mci *h, unsigned val)
{
	u32 reg = 0;

	if (val & 0x01)
		reg |= CONTROL3_FCSEL0;
	if (val & 0x02)
		reg |= CONTROL3_FCSEL1;

	return reg;
}

static void esdhc_speed_mode_set(struct esdhc_mci *h, unsigned freq)
{
	u8 reg = readb(h->base + HOSTCTL) & ~HOSTCTL_ENHIGHSPD;

	/*
	 * enable HC's high speed mode, when a card clock
	 * above 25 MHz should be used
	 */
	if (freq > 25000000)
		reg |= HOSTCTL_ENHIGHSPD;

	writeb(reg, h->base + HOSTCTL);
}

/* setup the physical features of the interface */
int esdhcp_interface_setup(struct esdhc_mci *h, unsigned freq)
{
	struct s3c_sdhc_platform_data *pd = to_s3c_pdata(h->pdata);
	u32 reg, control3 = readl(h->base + CONTROL3) & ~CONTROL3_FCSEL_MASK;

	esdhc_speed_mode_set(h, freq);

	/* TODO get more values from the platform */
	reg = CONTROL2_ENCLKOUTHOLD |
		CONTROL2_DEFCNT(2) |
		CONTROL2_ENSTAASYNCCLR |
		CONTROL2_ENCMDCNFMSK |
		CONTROL2_SELBASECLK(pd->clk_src);

	if (freq > 25000000) {
		pr_debug("Setting up the delays for frequencies above 25 MHz\n");
		if (pd->rx_delay_en25) {
			reg |= CONTROL2_ENFBCLKRX;
			control3 |= esdhc_rx_delay_set(h, pd->rx_delay25);
		}
		if (pd->tx_delay_en25) {
			reg |= CONTROL2_ENFBCLKTX;
			control3 |= esdhc_tx_delay_set(h, pd->tx_delay25);
		}
	} else {
		pr_debug("Setting up the delays for frequencies below 25 MHz\n");
		if (pd->rx_delay_en) {
			reg |= CONTROL2_ENFBCLKRX;
			control3 |= esdhc_rx_delay_set(h, pd->rx_delay);
		}
		if (pd->tx_delay_en) {
			reg |= CONTROL2_ENFBCLKTX;
			control3 |= esdhc_tx_delay_set(h, pd->tx_delay);
		}
	}

	writel(reg, h->base + CONTROL2);
	writel(control3, h->base + CONTROL3);

	/*
	 * Pay attention: on the S3C6410 SoC the third channel
	 * cannot be set in this way!
	 */
	writel(CONTROL4_SELCLKPADDS(pd->pin_strength), h->base + CONTROL4);
	return 0;
}

static int esdhcp_reset_wait_done(struct esdhc_mci *h)
{
	struct device_d *dev = h->mci.hw_dev;
	int rc;

	rc = wait_on_timeout(1 * SECOND,
			!(readl(h->base + CONTROL2) & CONTROL2_HWINITFIN));
	if (rc != 0)
		dev_err(dev, "Cannot reset the ESDHC\n");

	return rc;
}

int esdhcp_initialize(struct esdhc_mci *h)
{
	int rc;

	/* reset the engine */
	writeb(SWRST_RSTCMD | SWRST_RSTALL, h->base + SWRST);
	rc = esdhcp_reset_wait_done(h);
	if (rc != 0)
		return rc;

	/* TODO right? */
	writeb(0xe, h->base + PWRCON);	/* Set 3.3 V */
	writeb(0xf, h->base + PWRCON);	/* enable power */

	return 0;
}

static unsigned esdhcp_get_base_clk(struct esdhc_mci *h, unsigned freq)
{
	struct device_d *dev = h->mci.hw_dev;
	struct s3c_sdhc_platform_data *pd = to_s3c_pdata(h->pdata);

	switch (pd->clk_src) {
	case 0:
	case 1:
		pr_debug("%s: Using hclk as the input clock\n", __func__);
		return s3c_get_hclk();
	case 2:
		pr_debug("%s: Using HSMMCLK as the input clock\n", __func__);
		return s3c_get_hsmmc_clk(dev->id);
	default:
		dev_err(dev, "External SDHCI clock not supported\n");
		return 0;
	}
}

static void esdhcp_use_new_clock(struct esdhc_mci *h, unsigned div)
{
	struct device_d *dev = h->mci.hw_dev;
	int rc;

	/* stop the clock prior changing its frequency */
	esdhc_disable_SD_clock(h);

	/* setup HC local clock settings and enable engine's clock */
	writew(CLKCON_SELFREQ(div) | CLKCON_ENINTCLK, h->base + CLKCON);

	rc = wait_on_timeout(1 * SECOND, readw(h->base + CLKCON) & CLKCON_STBLINTCLK);
	if (rc == 0)
		esdhc_enable_SD_clock(h);
	else
		dev_warn(dev, "internal clock stabilization failed\n");
}

/*
 * Find a new baseclock best matching the requested card frequency
 *
 * We can use:
 * - EPLLout (mostly 24 MHz, via HSMMCx)
 * - MPLLout (mostly 266 MHz, via HSMMCx)
 * - EPLLin  (mostly 12 MHz, via HSMMCx)
 * - 27 MHz  (guess what, via HSMMCx)
 * - HCLK (mostly 133 MHz)
 * - external clock (mostly not connected)
 *
 * There are many MMC related clocks mentionend in the datasheet.
 * By trial and error I found:
 *
 *         MMCx_SEL
 * EPLLout ---0-\
 * MPLLout ---1--\-----SCLK_MMCx----DIV_MMCx--+---->HSMMCx
 * EPLLin  ---2--/       on/off      1..16    |
 * 27 MHz  ---3-/                             |  SELBASECLK
 *                                            |      v
 * HCLK --HCLK_GATE---o-----------------------|--o---0--\
 *                    |                       |  +---1---\--ESDHCDIV--> SD card
 *                    |                       +------2---/               clock
 *                    |      EXTCLK -----------------3--/
 *                    |
 *                    +--------------------------------->| MMC unit clock
 */
unsigned esdhcp_setup_clock(struct esdhc_mci *h, unsigned freq)
{
	struct device_d *dev = h->mci.hw_dev;
	unsigned u, bclk, tclk;

	if (freq == 0)
		return 0;

	dev_dbg(dev, "%s: Should set %u Hz.\n", __func__, freq);

	bclk = esdhcp_get_base_clk(h, freq);
	dev_dbg(dev, "%s: Starting with %d Hz.\n", __func__, bclk);

	if (freq > bclk) {
		esdhcp_use_new_clock(h, 0);
		return bclk;
	}

	/* calculate a divider according to manual page 993 (SELFREQ) */
	for (u = 1; u <= 0x80; u <<= 1) {
		tclk = bclk / (u << 1);
		if (freq >= tclk) {
			esdhcp_interface_setup(h, tclk);
			esdhcp_use_new_clock(h, u);
			dev_dbg(dev, "Using %d Hz", tclk);
			if (tclk != freq)
				pr_debug(" instead of %d\n", freq);
			else
				pr_debug("\n");
			return tclk;
		}
	}

	dev_err(dev, "Used physical clock is too high. Trying to continue\n");
	esdhcp_interface_setup(h, bclk / 256);
	esdhcp_use_new_clock(h, 0x80);
	return tclk;
}

/* ----------------------------------------------------------------------- */


#define CLK_SRC 0x1C/* 0x7E00F01c */
/* 18/19 + 0 EPLLout, 1 MPLLout, 2 EPLLin, 3 27 MHz*/

#define HCLK_GATE 0x30/* 0x7E00F030 */
/* 17 + "gating HCLK for HSMMCx" */

#define SCLK_GATE 0x38 /* 0x7E00F038 */
/* 27 + und 24 + ??????? "gating special clock for MMCx" */

#define CLK_DIV1 0x24 /* 0x7E00F024 */
/* 0 + 4 + 8 */

/* HCLK versorgt den Teil, der am internen Bus hängt */
/* dann gibt es noch auf Seite 953 ein BaseCLK für die SD-Karten-Seite */

/* die Bits im CONTROL2-Register nennen sich "Base Clock Source Select" */
/* 00 or 01 =HCLK, 10=HSMMCx (from SYSCON), 11=External
   Clock source (XTI or XEXTCLK) */

/*
 * There are many MMC related clocks mentionend in the datasheet.
 * By trial and error I found:
 *
 *         MMCx_SEL
 * EPLLout ---0-\
 * MPLLout ---1--\-----SCLK_MMCx----DIV_MMCx--+---->HSMMCx
 * EPLLin  ---2--/       on/off      1..16    |
 * 27 MHz  ---3-/                             |  SELBASECLK
 *                                            |      v
 * HCLK --HCLK_GATE---o-----------------------|--o---0--\
 *                    |                       |  +---1---\---ESDHCDIV--> SD card clock
 *                    |                       +------2---/
 *                    |      EXTCLK -----------------3--/
 *                    |
 *                    |
 *                    |
 *                    +--------------------------------->| MMC unit clock
 *
 * HCLK_HSMMC0 switches off the access to the ESDHC unit and the SD card clock
 * SCLK_MMC0 and SCLK_MMC0_48 does not change anything.
 * Changing the divider value does not change at least anything at the SD card clock
 * Changing the clock source settings, doesn't also change anything at the SD card clock
 *
 * There are some other bits and dividers.
 *
 * Versuche mit
 * - EPLLout, am SD Clock kam raus: 93.757 kHz -> EPLLout = 24 MHz
 * - MPLLout, am SD Clock kam raus: 1.03914 MHz -> MPLLout = 266 MHz
 * - EPLLin, am SD Clock kam raus: 46.8785 kHz -> EPLLin = 12 MHz
 * - 27 MHz, am SD Clock kam raus: 105.469 kHz -> 27 MHz
 *
 *  CLK_SRC 1 from 4 ------------- SCLK_MMCx -----DIV_MMCx-----> 2nd SELBASECLK
 *  ??? ----------- SCLK_MMCx_48 ---------> ?????
 *
 * Beim S5PV210 sind die Takte im CONTROL2 0/1 HCLK, 2 = SCLK_MMCx, 3 nix
 * -> damit sind sie beim S3C6410 genauso
 *
 * CONTROL2 0x7c200080
 */

static int esdhcp_s3c64xx_setup_input_clock(struct esdhc_mci *h, int id)
{
	struct s3c_sdhc_platform_data *pd = to_s3c_pdata(h->pdata);

	if (pd->clk_src == 2)	/* HSMMCx? */
		/* use the EPLL output as our source for the highest available clock */
		s3c_set_hsmmc_clk(id, 0, 1);

	return 0;
}

static int esdhc_s3c64xx_probe(struct device_d *dev)
{
	struct s3c_sdhc_platform_data *pd = dev->platform_data;
	struct esdhc_mci *this;

	if (pd == NULL) {
		dev_err(dev, "Missing platform data\n");
		return -EINVAL;
	}

	this = xzalloc(sizeof(struct esdhc_mci));

	dev->priv = this;
	this->pdata = &pd->esdhc_pd;

	this->base = dev_request_mem_region(dev, 0);
	this->mci.hw_dev = dev;

	/* enable the defined clock source */
	esdhcp_s3c64xx_setup_input_clock(this, dev->id);

	/* feed forward the platform specific values */
	this->mci.voltages = pd->esdhc_pd.voltages;
	this->mci.host_caps = pd->esdhc_pd.host_caps;

	this->mci.f_min = pd->esdhc_pd.f_min == 0 ? S5PCXX_CLOCK_REFERENCE / 256 : pd->esdhc_pd.f_min;
	this->mci.f_max = pd->esdhc_pd.f_max == 0 ? 52000000 : pd->esdhc_pd.f_max;

	pr_debug("%s: using f_min = %u Hz, f_max = %u Hz\n", __func__, this->mci.f_min, this->mci.f_max);

	return esdhc_register(this);
}

static struct driver_d esdhc_s3c64xx_driver = {
        .name  = "esdhc_s3c64xx",
        .probe = esdhc_s3c64xx_probe,
#ifdef CONFIG_MCI_INFO
	.info = esdhc_info,
#endif
};

static int esdhc_s3c64xx_init_driver(void)
{
        platform_driver_register(&esdhc_s3c64xx_driver);
        return 0;
}

device_initcall(esdhc_s3c64xx_init_driver);
