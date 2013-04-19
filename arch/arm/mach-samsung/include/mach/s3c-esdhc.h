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
 */

#ifndef __MCI_SDHC_H_
# define __MCI_SDHC_H_

#include <mci/esdhc.h>

struct s3c_sdhc_platform_data {
	struct esdhc_platform_data esdhc_pd;
	unsigned clk_src:2; /* 0/1 HCLK, 2 HSMMC, 3 external */
	unsigned pin_strength:2; /* 2mA, 4mA, 7mA or 9mA */
	unsigned pin_strength25:2; /* 2mA, 4mA, 7mA or 9mA above 25 MHz */
	unsigned rx_delay_en:1; /* true if rx_delay is valid */
	unsigned rx_delay:2; /* delay settings below 25 MHz */
	unsigned rx_delay_en25:1; /* true if rx_delay25 is valid */
	unsigned rx_delay25:2; /* delay settings above 25 MHz */
	unsigned tx_delay_en:1; /* true if tx_delay is valid */
	unsigned tx_delay:2; /* delay settings below 25 MHz */
	unsigned tx_delay_en25:1; /* true if tx_delay25 is valid */
	unsigned tx_delay25:2; /* delay settings above 25 MHz */
};

#endif /* __MCI_SDHC_H_ */
