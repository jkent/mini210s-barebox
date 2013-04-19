/*
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

#ifndef __MCI_ESDHC_H__
# define __MCI_ESDHC_H__

#include <mci.h>

struct esdhc_platform_data {
	unsigned voltages;
	unsigned host_caps;	/**< Host's interface capabilities, refer MMC_VDD_* */
	unsigned f_min;		/**< host interface lower limit */
	unsigned f_max;		/**< host interface upper limit */

	unsigned cd_type:3;
#define ESDHC_CD_NONE 0		/* can't detect if a card is plugged in */
#define ESDHC_CD_PERMANENT 1	/* always assume a card is plugged in */
#define ESDHC_CD_CONTROLLER 2	/* read from the controller */
#define ESDHC_CD_GPIO 3		/* read from a GPIO (pin is low if card is plugged in) */
#define ESDHC_CD_GPIO_INV 4	/* read from a GPIO (pin is high if card is plugged in) */
	unsigned cd_gpio; /* GPIO number in case cd_type == ESDHC_CD_GPIO */

	unsigned wp_type:3;
#define ESDHC_WP_NONE 0		/* can't detect if a card is write protected */
#define ESDHC_WP_PERMANENT 1	/* always assume a card is writeable */
#define ESDHC_WP_CONTROLLER 2	/* read from the controller */
#define ESDHC_WP_GPIO 3		/* read from a GPIO (pin is low if card is write protected) */
#define ESDHC_WP_GPIO_INV 4	/* read from a GPIO (pin is high if card is write protected) */
	unsigned wp_gpio;
};

struct esdhc_mci {
	struct mci_host mci;
	struct esdhc_platform_data *pdata;
	void __iomem *base;
	int bus_width:2; /* 0 = 1 bit, 1 = 4 bit, 2 = 8 bit */
	unsigned clock;	/* current clock in Hz */
};

#endif /* __MCI_ESDHC_H__ */
