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

#ifndef MCI_ESDHC_CORE_H
# define MCI_ESDHC_CORE_H

#define SDMASYSAD 0x00	/* u32 */
#define BLKSIZE 0x04	/* u16 */
#define BLKCNT 0x06	/* u16 */
#define ARGMNT 0x08	/* u32 */
#define TRNMOD 0x0c	/* u16 */
# define TRNMOD_ENDMA (1 << 0)
# define TRNMOD_ENBLKCNT (1 << 1)
# define TRNMOD_ENACMD12 (1 << 2)
# define TRNMOD_RD1WT0 (1 << 4)
# define TRNMOD_MUL1SIN0 (1 << 5)
# define TRNMOD_CCSCON(x) ((x) << 8)

#define CMDREG 0x0e	/* u16 */
# define CMDREG_RSPTYP_NONE 0
# define CMDREG_RSPTYP_136B 1
# define CMDREG_RSPTYP_48B 2
# define CMDREG_RSPTYP_48B_BUSY 3
# define CMDREG_ENCMDCRC (1 << 3)
# define CMDREG_ENCMDIDC (1 << 4)
# define CMDREG_DATAPRNT (1 << 5)
# define CMDREG_CMDTYP_OTHER (0 << 6)
# define CMDREG_CMDTYP_SUS_CMD52 (1 << 6)
# define CMDREG_CMDTYP_RES_CMD52 (2 << 6)
# define CMDREG_CMDTYP_IOABRT (3 << 6)
# define CMDREG_CMDIDX(x) (((x) & 0x3f) << 8)

#define RSPREG0 0x10	/* u32 */
#define RSPREG1 0x14	/* u32 */
#define RSPREG2 0x18	/* u32 */
#define RSPREG3 0x1c	/* u32 */

#define BDATA 0x20	/* u32 */

#define PRNSTS 0x24	/* u32 */
# define PRNSTS_TCMD (1 << 24)
# define PRNSTS_GET_TDAT(x) (((x) >> 20) & 0xf)
# define PRNSTS_TCD (1 << 18)
# define PRNSTS_STBLCARD (1 << 17)
# define PRNSTS_INSCARD (1 << 16)
# define PRNSTS_BUFRDRDY (1 << 11)
# define PRNSTS_BUFWTRDY (1 << 10)
# define PRNSTS_RDTRANACT (1 << 9)
# define PRNSTS_WTTRANACT (1 << 9)
# define PRNSTS_CMDINHDAT (1 << 1)
# define PRNSTS_CMDINHCMD (1 << 0)

#define HOSTCTL 0x28	/* u8 */
# define HOSTCTL_WIDE4 (1 << 1)
# define HOSTCTL_ENHIGHSPD (1 << 2)
# define HOSTCTL_DMASEL (1 << 3) /* 2 bits */
# define HOSTCTL_WIDE8 (1 << 5) /* 2 bits */
#define PWRCON 0x29	/* u8 */
#define BLKGAP 0x2A	/* u8 */
#define WAKCON 0x2B	/* u8 */

#define CLKCON 0x2c	/* u16 */
# define CLKCON_SELFREQ(x) ((x) << 8)
# define CLKCON_SDCLKACTIVE (1 << 3)
# define CLKCON_ENSDCLK (1 << 2)
# define CLKCON_STBLINTCLK (1 << 1)
# define CLKCON_ENINTCLK (1 << 0)
#define TIMEOUTCON 0x2e	/* u8 */
#define SWRST 0x2f	/* u8 */
# define SWRST_RSTDAT (1 << 2)
# define SWRST_RSTCMD (1 << 1)
# define SWRST_RSTALL (1 << 0)
#define NORINTSTS 0x30	/* u16 */
# define NORINTSTS_STAERR (1 << 15)
# define NORINTSTS_STABUFRDRDY (1 << 5)
# define NORINTSTS_STABUFWTRDY (1 << 4)
# define NORINTSTS_STATRANCMPLT (1 << 1)
# define NORINTSTS_STACMDCMPLT (1 << 0)

#define NORINTSTSEN 0x34 /* u16 */

#define ERRINTSTS 0x32	/* u16 */
#define CAPAREG 0x40	/* u32 */
# define CAPAREG_CAPV18 (1 << 26)
# define CAPAREG_CAPV30 (1 << 25)
# define CAPAREG_CAPV33 (1 << 24)
# define CAPAREG_CAPDMA (1 << 22)
# define CAPAREG_CAPAHSPD (1 << 21)
# define CAPAREG_CAPMAXBLKLEN (3 << 16)
# define CAPAREG_GET_CAPMAXBLKLEN(x) (((x) >> 16) & 3)

#define HCVER 0xfe	/* u16 */
# define HCVER_GET_VENVER(x) ((x) >> 8)
# define HCVER_GET_SPECVER(x) ((x) & 0xff)

struct esdhc_mci;
struct device_d;

int esdhc_register(struct esdhc_mci *);
void esdhc_info(struct device_d *);
void esdhc_enable_SD_clock(struct esdhc_mci *);
void esdhc_disable_SD_clock(struct esdhc_mci *);

#endif /* MCI_ESDHC_CORE_H */
