/*
 * Copyright (C) 2015 TQ Systems
 * Author: Michael Krummsdorf <michael.krummsdorf@tq-group.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#ifndef __TQMA28_BB__
#define __TQMA28_BB__

#include <common.h>

#define	MUX_CONFIG_EMMC	(MXS_PAD_3V3 | MXS_PAD_4MA | MXS_PAD_PULLUP)
#define	MUX_CONFIG_SD	(MXS_PAD_3V3 | MXS_PAD_8MA | MXS_PAD_NOPULL)
#define	MUX_CONFIG_ENET	(MXS_PAD_3V3 | MXS_PAD_8MA | MXS_PAD_PULLUP)
#define	MUX_CONFIG_EMI	(MXS_PAD_3V3 | MXS_PAD_12MA | MXS_PAD_NOPULL)
#define	MUX_CONFIG_NOPU	(MXS_PAD_3V3 | MXS_PAD_8MA | MXS_PAD_NOPULL)
#define	MUX_CONFIG_PU	(MXS_PAD_3V3 | MXS_PAD_8MA | MXS_PAD_PULLUP)
void tqma28_bb_board_init_ll(const uint32_t arg, const uint32_t *resptr);

/* Defined in drivers/mmc/mxsmmc.c */
struct mxsmmc_priv {
	int			id;
	struct mxs_ssp_regs	*regs;
	uint32_t		buswidth;
	int			(*mmc_is_wp)(int);
	int			(*mmc_cd)(int);
	struct mxs_dma_desc	*desc;
};

#ifndef CONFIG_SPL_BUILD
int tqma28_bb_board_mmc_getwp(struct mmc *mmc);
int tqma28_bb_board_mmc_getcd(struct mmc *mmc);
int tqma28_bb_board_mmc_init(bd_t *bis);

int tqma28_bb_board_early_init_f(void);
int tqma28_bb_board_late_init(void);
#endif

/*
 * Device Tree Support
 */
#if defined(CONFIG_OF_BOARD_SETUP) && defined(CONFIG_OF_LIBFDT)
int tqma28_bb_ft_board_setup(void *blob, bd_t *bd);
#endif /* defined(CONFIG_OF_BOARD_SETUP) && defined(CONFIG_OF_LIBFDT) */

#endif
