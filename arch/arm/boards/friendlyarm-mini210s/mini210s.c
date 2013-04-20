/*
 * Copyright (C) 2012 Alexey Galakhov
 * Based on Mini6410 code by Juergen Beisert
 *
 * Copyright (C) 2012 Juergen Beisert, Pengutronix
 *
 * In some ways inspired by code
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * David Mueller, ELSOFT AG, <d.mueller@elsoft.ch>
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

#include <common.h>
#include <driver.h>
#include <init.h>
#include <sizes.h>
#include <generated/mach-types.h>
#include <dm9000.h>
#include <gpio.h>
#include <led.h>
#include <io.h>
#include <nand.h>
#include <sdhci.h>
#include <asm/armlinux.h>
#include <mach/s3c-iomap.h>
#include <mach/s3c-clocks.h>
#include <mach/s3c-generic.h>
#include <mach/s3c-busctl.h>

/*
 * dm9000 ethernet MAC/PHY
 * nCS1, EINT7, 16-bit
 */
static struct dm9000_platform_data dm9000_data = {
	.srom = 1,
};

/* sdhci0 */
static struct sdhci_platform_data sdhci0_data = {
	.max_clk = 52000000,
	.min_clk = 400000,
};

static const unsigned pin_usage[] = {
	/* DM9000 */
	MP011_CSn1,
	MP016_OEn,
	MP017_WEn,
};

static struct gpio_led leds[] = {
	{
		.gpio = GPJ20,
		.led = {
			.name = "led1",
		}
	}, {
		.gpio = GPJ21,
		.led = {
			.name = "led2",
		}
	}, {
		.gpio = GPJ22,
		.led = {
			.name = "led3",
		}
	}, {
		.gpio = GPJ23,
		.led = {
			.name = "led4",
		}
	}
};

static int mini210s_mem_init(void)
{
	arm_add_mem_device("ram0", S3C_SDRAM_BASE, s5p_get_memory_size());
	return 0;
}
mem_initcall(mini210s_mem_init);

static int mini210s_console_init(void)
{
	/*
	 * configure the UART1 right now, as barebox will
	 * start to send data immediately
	 */
	s3c_gpio_mode(GPA00_RXD0 | ENABLE_PU);
	s3c_gpio_mode(GPA01_TXD0);
	s3c_gpio_mode(GPA02_NCTS0 | ENABLE_PU);
	s3c_gpio_mode(GPA03_NRTS0);

	add_generic_device("s3c_serial", DEVICE_ID_DYNAMIC, NULL,
			   S3C_UART1_BASE, S3C_UART1_SIZE,
			   IORESOURCE_MEM, NULL);
	return 0;
}
console_initcall(mini210s_console_init);

static void mini210s_eth_init(void)
{
	uint32_t reg;

	/* Configure SROM bank1
	 *
	 * 16-bit, no wait
	 * Tacs/Tacc/Tach: 0ck/6ck/0ck
	 * Tcos/Tcoh:      0ck/0ck
	 * Tacp/PMC:       0ck/Normal
	 */
	reg = readl(S3C_BWSCON);
	reg &= ~0x000000f0;
	reg |=  0x00000010;
	writel(reg, S3C_BWSCON);
	writel(0x00050000, S3C_BANKCON1);

	add_dm9000_device(0, S3C_CS1_BASE, S3C_CS1_BASE + 8,
			IORESOURCE_MEM_16BIT, &dm9000_data);
}

static int mini210s_devices_init(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(pin_usage); i++)
		s3c_gpio_mode(pin_usage[i]);

	for (i = 0; i < ARRAY_SIZE(leds); i++) {
		leds[i].active_low = 1;
		gpio_direction_output(leds[i].gpio, leds[i].active_low);
		led_gpio_register(&leds[i]);
	}

	add_sdhci_device(0, S3C_SDI0_BASE, &sdhci0_data);
	//add_sdhci_device(2, S3C_SDI2_BASE, NULL);

	mini210s_eth_init();
	armlinux_set_bootparams((void*)S3C_SDRAM_BASE + 0x100);
	armlinux_set_architecture(MACH_TYPE_MINI210);

	return 0;
}
device_initcall(mini210s_devices_init);
