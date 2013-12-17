/*
 * Freescale MX28EVK board
 *
 * (C) Copyright 2011 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * Based on m28evk.c:
 * Copyright (C) 2011 Marek Vasut <marek.vasut@gmail.com>
 * on behalf of DENX Software Engineering GmbH
 *
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
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux-mx28.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <linux/mii.h>
#include <miiphy.h>
#include <netdev.h>
#include <errno.h>
#include <mmc.h>

DECLARE_GLOBAL_DATA_PTR;

/* Defined in drivers/mmc/mxsmmc.c */
struct mxsmmc_priv {
	int			id;
	struct mxs_ssp_regs	*regs;
	uint32_t		buswidth;
	int			(*mmc_is_wp)(int);
	int			(*mmc_cd)(int);
	struct mxs_dma_desc	*desc;
};

static uint16_t tqma28_emmc_dsr = 0x0100;

/*
 * Functions
 */
int board_early_init_f(void)
{
	/* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);

	/* SSP0 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);
	/* SSP1 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK1, 96000, 0);

#ifdef	CONFIG_CMD_USB
	/* MBa28 has two USB ports - power up both */
	mxs_iomux_setup_pad(MX28_PAD_SSP2_SS2__USB0_OVERCURRENT);
	mxs_iomux_setup_pad(MX28_PAD_AUART2_TX__GPIO_3_9 |
			MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL);
	gpio_direction_output(MX28_PAD_AUART2_TX__GPIO_3_9, 1);

	mxs_iomux_setup_pad(MX28_PAD_SSP2_SS1__USB1_OVERCURRENT);
	mxs_iomux_setup_pad(MX28_PAD_AUART2_RX__GPIO_3_8 |
			MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL);
	gpio_direction_output(MX28_PAD_AUART2_RX__GPIO_3_8, 1);
#endif

	return 0;
}

int dram_init(void)
{
	return mxs_dram_init();
}

int board_init(void)
{
	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	return 0;
}

#ifdef	CONFIG_CMD_MMC

unsigned tqma28_get_mmc_devid()
{
	unsigned long boot_mode;
	unsigned mmc_id;

	boot_mode = (*(volatile unsigned int *)(GLOBAL_BOOT_MODE_ADDR)) & 0xF;

	/*
	 * Handle bug in combination with Freescale workaround
	 * for ssp clock polarity issue:
	 * When boot mode not set to sd card, look for environment on mmmc.
	 * Booting spi/i2c devices thus loads emmc environment.
	 */
	if (boot_mode == BOOT_MODE_SD1)
		mmc_id = CONFIG_SD_INDEX;
	else
		mmc_id = CONFIG_MMC_INDEX;

	return mmc_id;
}

static int tqma28_sd_wp(int id)
{
	if (id != CONFIG_SD_INDEX) {
		printf("MXS MMC: Invalid card selected (card id = %d)\n", id);
		return 1;
	}

	return gpio_get_value(MX28_PAD_GPMI_RESETN__GPIO_0_28);
}

static int tqma28_sd_cd(int id)
{
	struct mmc *mmc = find_mmc_device(id);
	struct mxs_ssp_regs *ssp_regs = ((struct mxsmmc_priv *)(mmc->priv))->regs;

	if (mmc->block_dev.removable)
		return !(readl(&ssp_regs->hw_ssp_status) &
				SSP_STATUS_CARD_DETECT);
	else
		return -1;
}

int board_mmc_init(bd_t *bis)
{
	int ret = 0;
	struct mmc *mmc;

	ret = mxsmmc_initialize(bis, CONFIG_MMC_INDEX, NULL, tqma28_sd_cd) |
		mxsmmc_initialize(bis, CONFIG_SD_INDEX,
					tqma28_sd_wp,
					tqma28_sd_cd);

	mmc = find_mmc_device(CONFIG_MMC_INDEX);
	if (!mmc)
		printf("%s: MMC device %d not found.\n",
					__func__,
					CONFIG_MMC_INDEX);
	else {
		mmc->block_dev.removable = 0;
		mmc_set_dsr(mmc, tqma28_emmc_dsr);
	}

	/* Configure SD WP as input */
	gpio_direction_input(MX28_PAD_GPMI_RESETN__GPIO_0_28);

	return ret;
}
#endif

#ifdef	CONFIG_CMD_NET

int board_eth_init(bd_t *bis)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	struct eth_device *dev;
	int ret;

	ret = cpu_eth_init(bis);

	/* MX28EVK uses ENET_CLK PAD to drive FEC clock */
	writel(CLKCTRL_ENET_TIME_SEL_RMII_CLK | CLKCTRL_ENET_CLK_OUT_EN,
					&clkctrl_regs->hw_clkctrl_enet);

	/* Reset FEC PHYs */
	gpio_direction_output(MX28_PAD_ENET0_RX_CLK__GPIO_4_13, 1);
	udelay(50);
	gpio_set_value(MX28_PAD_ENET0_RX_CLK__GPIO_4_13, 0);
	udelay(200);
	gpio_set_value(MX28_PAD_ENET0_RX_CLK__GPIO_4_13, 1);

	ret = fecmxc_initialize_multi(bis, 0, 0, MXS_ENET0_BASE);
	if (ret) {
		puts("FEC MXS: Unable to init FEC0\n");
		return ret;
	}

	ret = fecmxc_initialize_multi(bis, 1, 3, MXS_ENET1_BASE);
	if (ret) {
		puts("FEC MXS: Unable to init FEC1\n");
		return ret;
	}

	dev = eth_get_dev_by_name("FEC0");
	if (!dev) {
		puts("FEC MXS: Unable to get FEC0 device entry\n");
		return -EINVAL;
	}

	dev = eth_get_dev_by_name("FEC1");
	if (!dev) {
		puts("FEC MXS: Unable to get FEC1 device entry\n");
		return -EINVAL;
	}

	return ret;
}

#endif

int tqma28_setup_rtc_clocksource(void)
{
#define HW_RTC_PERSISTENT0			(0x00000060)
#define BM_RTC_PERSISTENT0_XTAL32_FREQ		(0x00000040)
#define BM_RTC_PERSISTENT0_XTAL32KHZ_PWRUP	(0x00000020)
#define BM_RTC_PERSISTENT0_CLOCKSOURCE		(0x00000001)
	struct mxs_rtc_regs *rtc_regs = (struct mxs_rtc_regs *)MXS_RTC_BASE;
	uint32_t persistent0;

	persistent0 = readl(&rtc_regs->hw_rtc_persistent0);

	persistent0 |= BM_RTC_PERSISTENT0_XTAL32KHZ_PWRUP |
				BM_RTC_PERSISTENT0_CLOCKSOURCE;
	persistent0 &= ~BM_RTC_PERSISTENT0_XTAL32_FREQ;

	writel(persistent0, &rtc_regs->hw_rtc_persistent0);

	printf("RTC: 32KHz xtal (persistent0 0x%08X)\n", persistent0);

	return 0;
}

int misc_init_r(void)
{
	char *s = getenv("serial#");
	char *mmccmd[2];

	if (s && s[0]) {
		puts("Ser#:  ");
		puts(s);
	}
	putc('\n');

	mmccmd[0] = "mmc dev 0";
	mmccmd[1] = "mmc dev 1";

	run_command_list(mmccmd[CONFIG_SYS_MMC_ENV_DEV], -1, 0);
	setenv("mmcdev", CONFIG_SYS_MMC_ENV_DEV?"1":"0");

	tqma28_setup_rtc_clocksource();

	return 0;
}
