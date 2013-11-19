/*
 * cmd_otp_mxs.c - interface to MXS on-chip One-Time-Programmable memory
 *
 * Copyright (c) 2013 TQ Systems GmbH
 *
 * Licensed under the GPL-2 or later.
 */

#include <config.h>
#include <common.h>
#include <command.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>

#define dbg(...)

static int chartomac(char *string, u8 *mac)
{
	char *cur, *prev;
	uint8_t i, ret;

	dbg("%s: string@%p=%s\n",__func__,string,string);
	ret = 0;
	i = 0;
	memset(mac, 0, 6);

	for (cur = string; *cur != '\0'; cur++)
	{
		prev = cur;
		mac[i] = simple_strtoul(cur, &cur, 16);
		dbg("%s: cur[%c|%p] prev[%c|%p] i[%d]\n",
			__func__,*cur,cur,*prev,prev,i);
		dbg("%s: mac[%x:%x:%x:%x:%x:%x]\n",
			__func__,mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

		/* moved ahead 2 chars? */
		if (prev+2 == cur)
			i++;

		if (i == 6)
			break;
	}

	if (i < 6)
		ret = 1;

	return ret;
}

static int mxs_hclk_to_mhz(uint32_t *hclk_old, uint32_t target_mhz)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	uint32_t clkctrl;
	uint32_t hclk, div_hclk;
	uint32_t div_target;

	hclk = mxc_get_clock(MXC_AHB_CLK);
	if (!hclk) {
		printf("Error in getting hclk.\n");
		return -1;
	}
	if (hclk_old)
		*hclk_old = hclk / 1000000;

	clkctrl = readl(&clkctrl_regs->hw_clkctrl_hbus);

	/* No support of fractional divider calculation */
	if (clkctrl & CLKCTRL_HBUS_DIV_FRAC_EN)
		return 0;

	div_hclk = clkctrl & CLKCTRL_HBUS_DIV_MASK;
	div_target = DIV_ROUND_UP(hclk,
		((target_mhz * 1000000) / div_hclk));

	while(readl(&clkctrl_regs->hw_clkctrl_hbus) & CLKCTRL_HBUS_ASM_BUSY)
		;

	dbg("Setting hbus, div = %d\n", div_target);

	writel(div_target << CLKCTRL_HBUS_DIV_OFFSET,
			&clkctrl_regs->hw_clkctrl_hbus);

	while(readl(&clkctrl_regs->hw_clkctrl_hbus) & CLKCTRL_HBUS_ASM_BUSY)
		;

	dbg("clock is now: %d\n", mxc_get_clock(MXC_AHB_CLK));

	return 0;
}

static int mxs_set_vddio(uint32_t *vddio_old, uint32_t vddio_mv)
{
	struct mxs_power_regs *power_regs =
		(struct mxs_power_regs *)MXS_POWER_BASE;
	uint32_t vddioctrl;

	vddioctrl = readl(&power_regs->hw_power_vddioctrl);
	if (vddio_old)
		*vddio_old = ((vddioctrl & POWER_VDDIOCTRL_TRG_MASK) * 50) + 2800;

	vddioctrl = vddioctrl & ~POWER_VDDIOCTRL_TRG_MASK;
	vddio_mv = (((vddio_mv - 2800) * 2) / 100);

	vddioctrl = vddioctrl | vddio_mv;

	dbg("Setting vddio to %d mV\n", ((vddio_mv * 50) + 2800));
	writel(vddioctrl, &power_regs->hw_power_vddioctrl);

	return 0;
}

#define MXS_OCOTP_MAX_TIMEOUT 1000000
static int otp_mxs_mactofuse(unsigned char *mac)
{
	struct mxs_ocotp_regs *ocotp_regs =
		(struct mxs_ocotp_regs *)MXS_OCOTP_BASE;
	uint32_t hclk_old, vddio_old;
	uint32_t data;
	uint32_t busy = 0;

	mxs_hclk_to_mhz(&hclk_old, 24);

	/* vddio to 2.8V using hw_power_vddioctrl_trg */
	mxs_set_vddio(&vddio_old, 2800);

	/* close banks for reading */
	writel(OCOTP_CTRL_RD_BANK_OPEN, &ocotp_regs->hw_ocotp_ctrl_clr);

	/* ocotp_ctrl_busy & ctrl_error & rd_bank_open clear? */
	busy = mxs_wait_mask_clr(&ocotp_regs->hw_ocotp_ctrl_reg,
				OCOTP_CTRL_BUSY, MXS_OCOTP_MAX_TIMEOUT);
	busy += mxs_wait_mask_clr(&ocotp_regs->hw_ocotp_ctrl_reg,
				OCOTP_CTRL_ERROR, MXS_OCOTP_MAX_TIMEOUT);
	busy += mxs_wait_mask_clr(&ocotp_regs->hw_ocotp_ctrl_reg,
				OCOTP_CTRL_RD_BANK_OPEN, MXS_OCOTP_MAX_TIMEOUT);
	if (busy) {
		printf("OCOTP Controller not ready for programming.\n");
		return -1;
	}

	/* wait at least 2 us */
	udelay(5);

	/* write req. address (cust0 = 0x00) to ctrl_addr */
	/* write unlock code to ctrl_wr_unlock (see register description)) */
	writel(OCOTP_CTRL_WR_UNLOCK_MASK | OCOTP_CTRL_ADDR_MASK,
		&ocotp_regs->hw_ocotp_ctrl_clr);

	writel(OCOTP_CTRL_WR_UNLOCK_KEY | 0x00,
		&ocotp_regs->hw_ocotp_ctrl_set);

	/* write mac to hw_ocotp_data */
	data = mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5];

	printf("Writing %08X to HW_OCOTP_CUST0... ", data);
	writel(data, &ocotp_regs->hw_ocotp_data);

	/* wait until ctrl_busy is clear */
	mxs_wait_mask_clr(&ocotp_regs->hw_ocotp_ctrl_reg,
				OCOTP_CTRL_BUSY, MXS_OCOTP_MAX_TIMEOUT);

	printf("done.\n");

	/* wait 2 us (hw_digctrl_microseconds)*/
	udelay(5);

	/* ctrl_error set? */
	if (readl(&ocotp_regs->hw_ocotp_ctrl_reg) & OCOTP_CTRL_ERROR)
		printf("Warning: Error bit set!\n");

	/* everything done -> reset system vddio & hclk */
	mxs_set_vddio(NULL, vddio_old);
	mxs_hclk_to_mhz(NULL, hclk_old);

	return 0;
}

static int otp_mxs_getmac(void)
{
	struct mxs_ocotp_regs *ocotp_regs =
		(struct mxs_ocotp_regs *)MXS_OCOTP_BASE;
	uint32_t busy = 0;
	uint32_t data;
	u8 mac[6];
	char string[20];

	busy = mxs_wait_mask_clr(&ocotp_regs->hw_ocotp_ctrl_reg,
				OCOTP_CTRL_BUSY, MXS_OCOTP_MAX_TIMEOUT);
	busy += mxs_wait_mask_clr(&ocotp_regs->hw_ocotp_ctrl_reg,
				OCOTP_CTRL_ERROR, MXS_OCOTP_MAX_TIMEOUT);
	if (busy) {
		printf("OCOTP Controller not ready for reading.\n");
		return -1;
	}

	/* open banks for reading */
	writel(OCOTP_CTRL_RD_BANK_OPEN, &ocotp_regs->hw_ocotp_ctrl_set);

	/* wait until ctrl_busy is clear */
	mxs_wait_mask_clr(&ocotp_regs->hw_ocotp_ctrl_reg,
				OCOTP_CTRL_BUSY, MXS_OCOTP_MAX_TIMEOUT);

	data = readl(&ocotp_regs->hw_ocotp_cust0);

	mac[0] = 0x00;
	mac[1] = 0xD0; /* 00:D0:93 => TQ Components Gmbh */
	mac[2] = (data >> 24) & 0xff;
	mac[3] = (data >> 16) & 0xff;
	mac[4] = (data >> 8) & 0xff;
	mac[5] = data & 0xff;

	snprintf(string, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
		mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
	setenv("ethaddr", string);

	data = data + 1;
	mac[3] = (data >> 16) & 0xff;
	mac[4] = (data >> 8) & 0xff;
	mac[5] = data & 0xff;

	snprintf(string, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
		mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
	setenv("eth1addr", string);

	/* close banks after reading */
	writel(OCOTP_CTRL_RD_BANK_OPEN, &ocotp_regs->hw_ocotp_ctrl_clr);

	return 0;
}

int do_otp_mxs(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	uint32_t ret;
	bool force, write_data;
	u8 mac[6];

	if (argc < 2) {
usage:
		return CMD_RET_USAGE;
	}

	write_data = false;
	force = false;
	if (!strncmp(argv[1], "getmac", 7))
		write_data = false;
	else if (!strncmp(argv[1], "mactofuse", 10)) {
		if (argc < 3)
			goto usage;

		if (!strncmp(argv[2], "--force", 8)) {
			force = true;
			ret = chartomac(argv[3], mac);
		} else
			ret = chartomac(argv[2], mac);

		if (ret || !is_valid_ether_addr(mac)) {
			printf("Error: '%s' is no valid MAC address.\n", argv[3]);
			return CMD_RET_FAILURE;
		}

		write_data = true;
	} else
		goto usage;

	/* do to the nature of OTP, make sure users are sure */
	if (write_data && !force) {
		printf(
			"Writing one time programmable memory.\n"
			"  MAC: %pM\n"
			"Bytes 2+3 identify vendor.\n"
			"Will write bytes 3-6 to fuse register.\n"
			"First 2 bytes will be assumed as 00:D0.\n"
			"type YES to confirm (case-sensitive): ",
			mac
		);

		uint8_t i = 0;
		while (1) {
			if (tstc()) {
				const char exp_ans[] = "YES\r";
				char c;
				putc(c = getc());
				if (exp_ans[i++] != c) {
					printf(" Aborting\n");
					return CMD_RET_FAILURE;
				} else if (!exp_ans[i]) {
					puts("\n");
					break;
				}
			}
		}
	}

	if (write_data)
		ret = otp_mxs_mactofuse(mac);
	else
		ret = otp_mxs_getmac();

	return ret;
}

U_BOOT_CMD(
	otp_mxs, 7, 0, do_otp_mxs,
	"MXS One-Time-Programmable sub-system",
	"getmac\n"
	" - read MAC address from OTP fuse and use for ETH0 device (ETH1 = ETH0+1)\n"
	"otp_mxs mactofuse [--force] <mac-address>\n"
	" - burn [without confirmation] 'mac-address' into the one-time writable fuses\n"
);
