/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 * Copyright (C) 2013 IEQualize GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef __HOMEBOX_H
#define __HOMEBOX_H

#define CONFIG_HOMEBOX

/* SoC configurations */
#define CONFIG_MX28				/* i.MX28 SoC */
#define CONFIG_TQMA28L				/* TQMa28L SoM */

#define CONFIG_MXS_GPIO				/* GPIO control */
#define CONFIG_SYS_HZ		1000		/* Ticks per second */

/* This should be removed after it's added into mach-types.h */
#define MACH_TYPE_HOMEBOX	3771

#define CONFIG_MACH_TYPE	MACH_TYPE_HOMEBOX

#include <asm/arch/regs-base.h>

#define CONFIG_SYS_NO_FLASH
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_ARCH_MISC_INIT
#define CONFIG_MISC_INIT_R

/* SPL */
#define CONFIG_SPL
#define CONFIG_SPL_NO_CPU_SUPPORT_CODE
#define CONFIG_SPL_START_S_PATH	"arch/arm/cpu/arm926ejs/mxs"
#define CONFIG_SPL_LDSCRIPT	"arch/arm/cpu/arm926ejs/mxs/u-boot-spl.lds"
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_GPIO_SUPPORT
#define CONFIG_SPL_MXS_NO_DCDC_BATT_SOURCE

/* U-Boot Commands */
#include <config_cmd_default.h>
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DOS_PARTITION

#define CONFIG_CMD_CACHE
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_GPIO
#define CONFIG_CMD_MII
#define CONFIG_CMD_MMC
#define CONFIG_CMD_NET
#define CONFIG_CMD_NFS
#define CONFIG_CMD_PING
#define CONFIG_CMD_SETEXPR
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_ITEST
#define CONFIG_CMD_UNZIP
#define CONFIG_CMD_UNXZ
#define CONFIG_XZ

/* Memory configurations */
#define CONFIG_NR_DRAM_BANKS		1		/* 1 bank of DRAM */
#define PHYS_SDRAM_1			0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE		0x40000000	/* Max 1 GB RAM */
#define CONFIG_SYS_MALLOC_LEN		0x00400000	/* 4 MB for malloc */
#define CONFIG_SYS_MEMTEST_START	0x40000000	/* Memtest start adr */
#define CONFIG_SYS_MEMTEST_END		0x40400000	/* 4 MB RAM test */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1
/* Point initial SP in SRAM so SPL can use it too. */

#define CONFIG_SYS_INIT_RAM_ADDR	0x00000000
#define CONFIG_SYS_INIT_RAM_SIZE	(128 * 1024)

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/*
 * We need to sacrifice first 4 bytes of RAM here to avoid triggering some
 * strange BUG in ROM corrupting first 4 bytes of RAM when loading U-Boot
 * binary. In case there was more of this mess, 0x100 bytes are skipped.
 */
#define CONFIG_SYS_TEXT_BASE	0x40000100

#define CONFIG_ENV_OVERWRITE
/* U-Boot general configurations */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT	"> "
#define CONFIG_SYS_CBSIZE	1024		/* Console I/O buffer size */
#define CONFIG_SYS_PBSIZE	\
	(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
						/* Print buffer size */
#define CONFIG_SYS_MAXARGS	32		/* Max number of command args */
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE	/* Boot argument buffer size */
#define CONFIG_VERSION_VARIABLE		/* U-BOOT version */
#define CONFIG_AUTO_COMPLETE		/* Command auto complete */
#define CONFIG_CMDLINE_EDITING		/* Command history etc */
#define CONFIG_SYS_HUSH_PARSER

/* Serial Driver */
#define CONFIG_PL011_SERIAL
#define CONFIG_PL011_CLOCK		24000000
#define CONFIG_PL01x_PORTS		{ (void *)MXS_UARTDBG_BASE }
#define CONFIG_CONS_INDEX		0
#define CONFIG_BAUDRATE			115200  /* Default baud rate */

/* DMA */
#define CONFIG_APBH_DMA

/* MMC Driver */
#define CONFIG_MMC_INDEX 0
#define CONFIG_ENV_IS_IN_MMC
#ifdef CONFIG_ENV_IS_IN_MMC
 #define CONFIG_ENV_OFFSET		(0x400)			/*  1 KB */
 #define CONFIG_ENV_SIZE		(0x10000 - 0x400)	/* 63 KB */
 #define CONFIG_SYS_MMC_ENV_DEV		CONFIG_MMC_INDEX
 #define CONFIG_ENV_OFFSET_REDUND	(0x10400)		/* 65 KB */
 #define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#endif
#define CONFIG_CMD_SAVEENV
#ifdef CONFIG_CMD_MMC
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_MXS_MMC
#define CONFIG_CMD_EXT2                 /* EXT2 Support */
#define CONFIG_CMD_EXT4                 /* EXT4 Support */
#endif

/* Ethernet on SOC (FEC) */
#ifdef CONFIG_CMD_NET
#define CONFIG_ETHPRIME		"FEC"
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_FEC_XCV_TYPE	RMII
#if 0
#define CONFIG_MX28_FEC_MAC_IN_OCOTP /* use MAC from environment */
#endif
#define CONFIG_PHYLIB
#define CONFIG_FEC_MXC_PHYADDR	1
#define IMX_FEC_BASE		MXS_ENET0_BASE
#define CONFIG_PHY_SMSC
#define CONFIG_NETCONSOLE
#endif

#define CONFIG_IPADDR		192.168.9.100
#define CONFIG_SERVERIP		192.168.9.133
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_GATEWAYIP	192.168.9.254

/* BOOTP options */
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_HOSTNAME

/* Boot Linux */
#define CONFIG_CMDLINE_TAG
#define CONFIG_REVISION_TAG
#define CONFIG_SETUP_MEMORY_TAGS

#define CONFIG_BOOTDELAY		5	/* autoboot after 5 seconds */
#define CONFIG_BOOT_RETRY_TIME		120	/* retry autoboot after 120 seconds */
#define CONFIG_BOOT_RETRY_TIME_MIN	1	/* can go down to 1 second */
#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_AUTOBOOT_PROMPT		"Autobooting in %d seconds, " \
		"press <c> to stop\n", bootdelay
#define CONFIG_AUTOBOOT_DELAY_STR	"\x63"	/* allows retry after retry time */
#define CONFIG_AUTOBOOT_STOP_STR	" "	/* stop autoboot with <Space> */
#define CONFIG_RESET_TO_RETRY			/* reset board to retry booting */

#define CONFIG_BOOTFILE		"uImage"
#define CONFIG_BOOTCOMMAND	"run find_bootsys; run start_nc"
#define CONFIG_LOADADDR		0x42000000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR
#define CONFIG_OF_LIBFDT

/* Extra Environment */
#define CONFIG_EXTRA_ENV_SETTINGS \
		"rd_size=16384\0"                                               \
		"netdev=eth0\0"                                                 \
		"console_fsl=ttyAM0\0"                                          \
		"console_mainline=ttyAMA0\0"                                    \
		"kernel=uImage\0"                                               \
		"ethaddr=02:c0:1d:c0:ff:ee\0"                                   \
		"bootsys=1\0"                                                   \
		"linux1_start=1000\0"                                           \
		"linux2_start=2800\0"                                           \
		"addether=setenv bootargs fec_mac=${ethaddr}\0"                 \
		"addtty=setenv bootargs ${bootargs} console=${console_fsl},${baudrate}\0"         \
		"addmisc=setenv bootargs ${bootargs} tq_dsr=${tq_dsr} panic=1\0"                  \
		"erase_mmc=mw.b ${loadaddr} 0 512; mmc write ${loadaddr} 0 2\0"                   \
		"erase_env1=mw.b ${loadaddr} 0 512; mmc write ${loadaddr} 2 7E\0"                 \
		"erase_env2=mw.b ${loadaddr} 0 512; mmc write ${loadaddr} 80 7E\0"                \
		"addmmc=setenv bootargs ${bootargs} root=/dev/mmcblk0p3 rootwait\0"               \
		"addrootfs1=setenv bootargs ${bootargs} root=/dev/mmcblk0p5 rootwait bootsys=1\0" \
		"addrootfs2=setenv bootargs ${bootargs} root=/dev/mmcblk0p6 rootwait bootsys=2\0" \
		"mmc_sys1=run addether addrootfs1 addtty addmisc; "                               \
				"mmc read ${loadaddr} ${linux1_start} 1800; bootm\0"              \
		"mmc_sys2=run addether addrootfs2 addtty addmisc; "                               \
				"mmc read ${loadaddr} ${linux2_start} 1800; bootm\0"              \
		"find_bootsys=echo starting bootscript; "                                         \
			"if test -n ${bootsys}; then "                                            \
				"if test -n ${checksys}; then "                                   \
					"if test ${checksys} = 0; then "                          \
						"echo error booting the new system, "             \
						     "trying to boot the old one; "               \
						"if test ${bootsys} = 1; then "                   \
							"echo booting sys 2 && run mmc_sys2; "    \
						"else "                                           \
							"echo booting sys 1 && run mmc_sys1; "    \
						"fi; "                                            \
					"else "                                                   \
						"setenv checksys 0; saveenv; "                    \
					"fi; "                                                    \
				"fi; "                                                            \
				"if test ${bootsys} = 1; then "                                   \
					"echo booting sys 1 && run mmc_sys1 || "                  \
					"echo booting alt. sys 2 && run mmc_sys2; "               \
				"else "                                                           \
					"if test ${bootsys} = 2; then "                           \
						"echo booting sys 2 && run mmc_sys2 || "          \
						"echo booting alt. sys 1 && run mmc_sys1; "       \
					"fi; "                                                    \
				"fi; "                                                            \
			"else "                                                                   \
				"echo bootsys not set correctly; "                                \
			"fi\0"                                                                    \
		"serialconsole=setenv stderr serial; setenv stdout serial; setenv stdin serial\0" \
		"netconsole=setenv stderr nc; setenv stdout nc; setenv stdin nc\0"                \
		"netconsole_enabled=0\0"                                                          \
		"ncip=192.168.9.133\0"                                                            \
		"start_nc=if test ${netconsole_enabled} = 1; then "                               \
			"echo starting netconsole; run netconsole; version; fi\0"                 \
		"start_deploy="                                                                   \
			"mw.l ${loadaddr} efbeadde 80; "                                          \
			"mmc write ${loadaddr} 44030 1; "                                         \
			"tftp ${loadaddr} openwrt-mxs-homebox-root.ext4.xz; "                     \
			"unxz ${loadaddr} ${filesize} 43000000; "                                 \
			"setexpr fsize ${filesize} + 1ff; "                                       \
			"setexpr fsize ${fsize} / 200; "                                          \
			"mmc write 43000000 4010 ${fsize}; "                                      \
			"mmc write 43000000 24020 ${fsize}; "                                     \
			"tftp ${loadaddr} openwrt-mxs-homebox-uImage; "                           \
			"setexpr fsize ${filesize} + 1ff; "                                       \
			"setexpr fsize ${fsize} / 200; "                                          \
			"mmc write ${loadaddr} ${linux2_start} ${fsize}; "                        \
			"mmc write ${loadaddr} ${linux1_start} ${fsize}\0"

/*
 * The global boot mode will be detected by ROM code and
 * a boot mode value will be stored at fixed address:
 * TO1.0 addr 0x0001a7f0
 * TO1.2 addr 0x00019BF0
 */
#ifndef MX28_EVK_TO1_0
 #define GLOBAL_BOOT_MODE_ADDR 0x00019BF0
#else
 #define GLOBAL_BOOT_MODE_ADDR 0x0001a7f0
#endif
#define BOOT_MODE_SD0 0x9
#define BOOT_MODE_SD1 0xa

#endif /* __HOMEBOX_H */
