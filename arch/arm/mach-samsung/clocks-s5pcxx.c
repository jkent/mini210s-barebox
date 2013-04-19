/*
 * Copyright (C) 2012 Alexey Galakhov
 * Copyright (C) 2012 Juergen Beisert, Pengutronix
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

#include <config.h>
#include <common.h>
#include <init.h>
#include <clock.h>
#include <io.h>
#include <mach/s3c-iomap.h>
#include <mach/s3c-generic.h>
#include <mach/s3c-clocks.h>

static inline uint32_t clkdiv(uint32_t clk, unsigned bit, unsigned mask)
{
	uint32_t ratio = (readl(S5P_CLK_DIV0) >> bit) & mask;
	return clk / (ratio + 1);
}

uint32_t s3c_get_mpllclk(void)
{
	uint32_t m, p, s;
	uint32_t reg = readl(S5P_xPLL_CON + S5P_MPLL);
	m = (reg >> 16) & 0x3ff;
	p = (reg >> 8) & 0x3f;
	s = (reg >> 0) & 0x7;
	return m * ((S5PCXX_CLOCK_REFERENCE) / (p << s));
}

uint32_t s3c_get_apllclk(void)
{
	uint32_t m, p, s;
	uint32_t reg = readl(S5P_xPLL_CON + S5P_APLL);
	m = (reg >> 16) & 0x3ff;
	p = (reg >> 8) & 0x3f;
	s = (reg >> 0) & 0x7;
	s -= 1;
	return m * ((S5PCXX_CLOCK_REFERENCE) / (p << s));
}

static uint32_t s5p_get_a2mclk(void)
{
	return clkdiv(s3c_get_apllclk(), 4, 0x7);
}

static uint32_t s5p_get_moutpsysclk(void)
{
	if (readl(S5P_CLK_SRC0) & (1 << 24)) /* MUX_PSYS */
		return s5p_get_a2mclk();
        else
		return s3c_get_mpllclk();
}

uint32_t s3c_get_hclk(void)
{
	return clkdiv(s5p_get_moutpsysclk(), 24, 0xf);
}

uint32_t s3c_get_pclk(void)
{
	return clkdiv(s3c_get_hclk(), 28, 0x7);
}

/* This works for most but not for all clock sources. Be careful. */
static unsigned s5p_get_dev_clk(u32 srcreg, u32 divreg, unsigned shift)
{
	u32 clk;
	u32 sel = (srcreg >> shift) & 0xf;
	u32 div = (divreg >> shift) & 0xf;

	switch (sel) {
	case 0: /* XXTI */
		clk = S5PCXX_CLOCK_REFERENCE;
		break;
	case 1: /* XusbXTI */
		clk = 24000000;
		break;
	case 2: /* HDMI27M */
		clk = 27000000;
		break;
	case 6: /* MPLL */
		clk = s3c_get_mpllclk();
		break;
	case 7: /* EPLL */
		/* TODO clk = s3c_get_epllclk(); */
	case 8: /* VPLL */
		/* TODO clk = s3c_get_vpllclk(); */
	case 3: /* USBPHY0 */
	case 4: /* USBPHY1 */
	case 5: /* HDMIPHY */
		/* TODO */
	default: /* reserved */
		return 0; /* FIXME we cannot report error here... */
	}

	return clk / (div + 1);
}

unsigned s3c_get_uart_clk(unsigned src) {
	int id = 0; /* FIXME */
	return (src & 1)
		? s5p_get_dev_clk(readl(S5P_CLK_SRC4), readl(S5P_CLK_DIV4), S5P_CLK_SRCDIV_UART(id))
		: s3c_get_pclk();
}

unsigned s3c_get_hsmmc_clk(int id)
{
	u32 gate = readl(S5P_CLK_GATE_IP2);
	printf("hsmmc gate reg = 0x%08x\n", gate);

	if (!(gate & S5P_CLK_GATE_MMC(id)))
		return 0; /* disabled */

	printf("hsmmc clk is enabled\n");
	return s5p_get_dev_clk(readl(S5P_CLK_SRC4), readl(S5P_CLK_DIV4), S5P_CLK_SRCDIV_MMC(id));
}

void s3c_set_hsmmc_clk(int id, int src, unsigned div)
{
	/* FIXME use src and div here - probably needs refactoring */
	u32 reg;
	unsigned shift = S5P_CLK_SRCDIV_MMC(id);
	u32 mask = ~(0xf << shift);

	if (!div)
		div = 1;

	reg = readl(S5P_CLK_SRC4) & mask;
	reg |= 0;
	writel(reg, S5P_CLK_SRC4);

	reg = readl(S5P_CLK_DIV4) & mask;
	reg |= (div - 1) << shift;
	writel(reg, S5P_CLK_DIV4);

	/* calling this function implies enabling of the clock */
	writel(readl(S5P_CLK_GATE_IP2) | S5P_CLK_GATE_MMC(id), S5P_CLK_GATE_IP2);
}

int s5pcxx_dump_clocks(void)
{
	printf("refclk:  %7d kHz\n", S5PCXX_CLOCK_REFERENCE / 1000);
	printf("apll:    %7d kHz\n", s3c_get_apllclk() / 1000);
	printf("mpll:    %7d kHz\n", s3c_get_mpllclk() / 1000);
/*	printf("CPU:     %7d kHz\n", s3c_get_cpuclk() / 1000); */
	printf("hclk:    %7d kHz\n", s3c_get_hclk() / 1000);
	printf("pclk:    %7d kHz\n", s3c_get_pclk() / 1000);
	return 0;
}

late_initcall(s5pcxx_dump_clocks);
