/*
 * TQ Systems MBa28 starterkit for TQMa28L module
 * (C) Copyright 2015 TQ Systems GmbH
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
#include <malloc.h>

#include "tqma28l_bb.h"

DECLARE_GLOBAL_DATA_PTR;

static uint8_t tqma28_enet_clk_int = 0;

const char *tqma28l_bb_get_boardname(void)
{
	return "NONE";
}

/*
 * Functions
 */
#ifdef CONFIG_SPL_BUILD
void tqma28l_bb_board_init_ll(const uint32_t arg, const uint32_t *resptr)
{
}
#endif

int tqma28l_bb_board_early_init_f(void)
{
	return 0;
}

#ifdef	CONFIG_CMD_MMC
int tqma28l_bb_board_mmc_init(bd_t *bis)
{
	return mxsmmc_initialize(bis, CONFIG_SD_INDEX, NULL, NULL);
}
#endif

int board_eth_init(bd_t *bis)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	int ret;

	/* Check env for var to enable enet clk output */
	tqma28_enet_clk_int = (1 == getenv_yesno("enet_clk_internal"));

	ret = cpu_eth_init(bis);

	if (tqma28_enet_clk_int) {
		puts("[FEC MXS: Using internal ENET_CLK]\n");
		writel(CLKCTRL_ENET_TIME_SEL_RMII_CLK | CLKCTRL_ENET_CLK_OUT_EN,
				&clkctrl_regs->hw_clkctrl_enet);
	} else {
		puts("[FEC MXS: Using external ENET_CLK]\n");
		writel(CLKCTRL_ENET_TIME_SEL_RMII_CLK,
				&clkctrl_regs->hw_clkctrl_enet);
		clrbits_le32(&clkctrl_regs->hw_clkctrl_enet,
				CLKCTRL_ENET_CLK_OUT_EN);
	}

	return ret;
}

/*
 * When user wants enet_clk driven by external source
 * tell the kernel to not enable the internal driver.
 *
 * By default the internal driver is enabled in mainline device tree
 * so we need to remove it in case we want external source.
 */
struct clkref {
	u32 ref;
	u32 id;
};
void tqma28_dt_remove_enet_out(void *blob)
{
	const char *names_prop = "clock-names";
	const char *names;
	const char *n;
	char *n_names;
	char *n_n;

	const char *clks_prop = "clocks";
	const u32 *clocks;
	const u32 *c;
	u32 *n_clocks;
	u32 *n_c;
	u8 clks_size = (2* sizeof(u32));

	int mac0_ofs;
	int clocks_len, names_len;
	int ret;
	int n_clocks_len = 0;
	int n_names_len = 0;
	int word = 0;

	mac0_ofs = fdt_path_offset(blob, "/ahb@80080000/ethernet@800f0000");

	clocks = fdt_getprop(blob, mac0_ofs, clks_prop, &clocks_len);
	n_clocks = calloc(1, clocks_len);

	names = fdt_getprop(blob, mac0_ofs, names_prop, &names_len);
	n_names = calloc(1, names_len);

	/*
	 * parse clock names for enet_out to remove
	 * by creating new properties without enet_out
	 */
	n = names;
	n_n = n_names;
	c = clocks;
	n_c = n_clocks;
	while (n < names + names_len)
	{
		/* string + null byte*/
		word = (strlen(n)+1);
		if (strcmp(n, "enet_out")) {
			/* not string we look for, keep it in n_names */
			memcpy(n_n, n, word);
			n_n += word;
			n_names_len += word;

			/*
			 * keep also the corresponding clocks entry
			 * consisting of two u32 values.
			 * TODO: use struct?
			 */
			memcpy(n_c, c, clks_size);
			n_c++; n_c++;
			n_clocks_len += clks_size;
		}
		n += word;
		c++; c++;
	}

	ret = fdt_setprop(blob, mac0_ofs, clks_prop, n_clocks, n_clocks_len);
	ret |= fdt_setprop(blob, mac0_ofs, names_prop, n_names, n_names_len);
	if (ret)
		printf("fdt_setprop(): %s\n", fdt_strerror(ret));
}

/*
 * Device Tree Setup
 */
#if defined(CONFIG_OF_BOARD_SETUP) && defined(CONFIG_OF_LIBFDT)
int tqma28l_bb_ft_board_setup(void *blob, bd_t *bd)
{
	if (!tqma28_enet_clk_int)
		tqma28_dt_remove_enet_out(blob);

	return 0;
}
#endif /* defined(CONFIG_OF_BOARD_SETUP) && defined(CONFIG_OF_LIBFDT) */
