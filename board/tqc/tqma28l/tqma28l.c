/*
 * TQ Systems TQMa28L module
 * (C) Copyright 2013-2015 TQ Systems GmbH
 * Author: Michael Krummsdorf <michael.krummsdorf@tq-group.com>
 *
 * Based on m28evk.c:
 * Copyright (C) 2011 Marek Vasut <marek.vasut@gmail.com>
 * on behalf of DENX Software Engineering GmbH
 *
 * SPDX-License-Identifier:	GPL-2.0+
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

#include "tqma28l_bb.h"
#include "../common/tqc_emmc.h"

DECLARE_GLOBAL_DATA_PTR;

static const uint16_t tqma28_emmc_dsr = 0x0100;

/*
 * Functions
 */
static const char *tqma28l_get_boardname(void)
{
#if defined(CONFIG_TQMA28L_AA)
	return "TQMa28L-AA (i.MX287)";
#elif defined(CONFIG_TQMA28L_AB)
	return "TQMa28L-AB (i.MX283)";
#else
	return "TQMa28L (i.MX28?)";
#endif
}

int checkboard(void)
{
	printf("Board: %s on a %s\n", tqma28l_get_boardname(),
	       tqma28l_bb_get_boardname());
	return 0;
}

int board_early_init_f(void)
{
	/* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);

	/* SSP0 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);
	/* SSP1 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK1, 96000, 0);

	tqma28l_bb_board_early_init_f();

	return 0;
}

int board_init(void)
{
	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	return 0;
}

int dram_init(void)
{
	return mxs_dram_init();
}

void mx28_adjust_mac(int dev_id, unsigned char *mac)
{
	uint32_t data;

	/*
	 * TQ Components Gmbh:      00:D0:93:xx:xx:xx (default)
	 */

	mac[0] = 0x00;
	mac[1] = 0xD0;

	if (dev_id == 1) { /* Let MAC1 be MAC0 + 1 by default */
		data = (mac[3] << 16) | (mac[4] << 8) | mac[5];
		data = data + 1;
		mac[3] = (data >> 16) & 0xFF;
		mac[4] = (data >> 8) & 0xFF;
		mac[5] = data & 0xFF;
	}
}

#ifdef	CONFIG_CMD_MMC
unsigned tqma28l_get_env_dev(void)
{
	struct mxs_spl_data *data = (struct mxs_spl_data *)
		((CONFIG_SYS_TEXT_BASE - sizeof(struct mxs_spl_data)) & ~0xf);

	if (mxs_boot_modes[data->boot_mode_idx].boot_pads == 0x9)
		return CONFIG_SD_INDEX;

	return CONFIG_MMC_INDEX;
}

static int tqma28l_emmc_cd(int id)
{
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	/* Baseboard SD is connected to SSP0.
	 * Its id should be cur_dev_num=0, so init it first.
	 */
	tqma28l_bb_board_mmc_init(bis);

	if (mxsmmc_initialize(bis, CONFIG_MMC_INDEX, NULL, tqma28l_emmc_cd)) {
		printf("Error initializing e-MMC\n");
	} else {
		struct mmc *mmc = find_mmc_device(CONFIG_MMC_INDEX);
		if (mmc) {
			mmc->block_dev.removable = 0;
		}
	}

	return 0;
}

/* board-specific MMC card detection / modification */
void board_mmc_detect_card_type(struct mmc *mmc)
{
	struct mmc *emmc = find_mmc_device(CONFIG_MMC_INDEX);

	if (emmc != mmc)
		return;

	if (tqc_emmc_need_dsr(mmc) > 0)
		mmc_set_dsr(mmc, tqma28_emmc_dsr);
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

extern int do_mmcops(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
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

/*
 * Device Tree Setup
 */
#if defined(CONFIG_OF_BOARD_SETUP) && defined(CONFIG_OF_LIBFDT)
int ft_board_setup(void *blob, bd_t *bd)
{
	struct mmc *mmc = find_mmc_device(CONFIG_MMC_INDEX);

	fdt_shrink_to_minimum(blob);

	if (mmc && (tqc_emmc_need_dsr(mmc) > 0))
		tqc_ft_fixup_emmc_dsr(blob,
					 "/apb@80000000/apbh@80000000/ssp@80012000",
					tqma28_emmc_dsr);

	tqma28l_bb_ft_board_setup(blob, bd);

	return 0;
}
#endif /* defined(CONFIG_OF_BOARD_SETUP) && defined(CONFIG_OF_LIBFDT) */
