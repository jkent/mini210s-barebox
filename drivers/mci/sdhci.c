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
#include <mci.h>
#include <init.h>
#include <driver.h>
#include <sdhci.h>

struct sdhci_host {
	struct mci_host		mci;
	void __iomem		*base;
};

#define to_sdhci(mci)	container_of(mci, struct sdhci_host, mci)

static int sdhci_init(struct mci_host *mci, struct device_d *dev)
{
	struct sdhci_host *host = to_sdhci(mci);

	return -ENODEV;
}

static int sdhci_probe(struct device_d *dev)
{
	struct sdhci_host *host;

	host = xzalloc(sizeof(*host));

	host->mci.init = sdhci_init;
	host->mci.hw_dev = dev;

	host->base = dev_request_mem_region(dev, 0);

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

