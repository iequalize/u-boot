/*
 * (C) Copyright 2013-2015 TQ Systems GmbH
 * Author: Michael Krummsdorf <michael.krummsdorf@tq-group.com>
 *
 * Based on m28evk.h:
 * Copyright (C) 2011 Marek Vasut <marek.vasut@gmail.com>
 * on behalf of DENX Software Engineering GmbH
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __CONFIGS_TQMA28L_H__
#define __CONFIGS_TQMA28L_H__

#include <linux/kconfig.h>

#if !defined(__ASSEMBLY__)
extern unsigned tqma28l_get_env_dev(void);
#endif

/* System configurations */
#define CONFIG_MX28				/* i.MX28 SoC */
#define CONFIG_MACH_TYPE	MACH_TYPE_TQMA28

#include <asm/arch/regs-base.h>

/* U-Boot Commands */
#define CONFIG_SYS_NO_FLASH
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO
#define CONFIG_DOS_PARTITION
#define CONFIG_MISC_INIT_R

#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_CACHE
#define CONFIG_CMD_DATE
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_FAT
#define CONFIG_CMD_FUSE
#define CONFIG_CMD_GPIO
#define CONFIG_CMD_I2C	/* disabled on mx28evk due to sgtl5000 issues, git a49db3df7 */
#define CONFIG_CMD_MII
#define CONFIG_CMD_MMC
#define CONFIG_CMD_PING
#if 0
#define CONFIG_CMD_SPI /* cmd_spi uses full-duplex, not supported by mx28 */
#endif
#define CONFIG_CMD_USB

/* Memory configuration */
#define CONFIG_NR_DRAM_BANKS		1		/* 1 bank of DRAM */
#define PHYS_SDRAM_1			0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE		0x40000000	/* Max 1 GB RAM */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1

/* Environment */

#if defined(CONFIG_TQMA28L_BOOT_MMC)

/* Environment is in MMC */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SD_INDEX		0
#define CONFIG_MMC_INDEX	1
#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_OFFSET		(0x400) /* 1 KB */
#define CONFIG_ENV_SIZE			(0x20000 - 0x400) /* 127 KB */
#define CONFIG_SYS_MMC_ENV_DEV		tqma28l_get_env_dev()

#endif

/* FEC Ethernet on SoC */
#ifdef	CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_PHYLIB
#define CONFIG_MX28_FEC_MAC_IN_OCOTP
#endif

/* RTC */
#ifdef	CONFIG_CMD_DATE
#define	CONFIG_RTC_MXS
#endif

/* USB */
#ifdef	CONFIG_CMD_USB
#define CONFIG_EHCI_MXS_PORT0
#define CONFIG_EHCI_MXS_PORT1
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#define	CONFIG_USB_STORAGE
#define	CONFIG_USB_HOST_ETHER
#define	CONFIG_USB_ETHER_ASIX
#define	CONFIG_USB_ETHER_SMSC95XX
#endif

/* Boot Linux */
#define CONFIG_BOOTDELAY	1
#define CONFIG_BOOTFILE		"zImage"
#define CONFIG_LOADADDR		0x42000000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR
#define CONFIG_SYS_MXS_VDD5V_ONLY
#define CONFIG_FIT
#define CONFIG_OF_BOARD_SETUP

#ifdef CONFIG_TQMA28L_AA
#define CONFIG_TQMA28L_DTB	imx28-mba28l-aa.dtb
#else
#define CONFIG_TQMA28L_DTB	imx28-mba28l-ab.dtb
#endif

#ifdef CONFIG_MXS_AUART
#define CONFIG_CONSOLE_DEV	ttyAPP1
#else
#define CONFIG_CONSOLE_DEV	ttyAMA0
#endif

/* Extra Environment */
/*
 * TODO: We still have to find out the amount of sectors consumed
 * by the U-Boot image on sd card updates, so we can write the redundant copy
 * behind the first. This number must be updated also in the MBR.
 */
#define	CONFIG_EXTRA_ENV_SETTINGS \
	"rd_size=16384\0"						\
	"netdev=eth0\0"							\
	"console=" __stringify(CONFIG_CONSOLE_DEV) "\0"			\
	"kernel=zImage\0"						\
	"uboot=u-boot.sb\0"						\
	"dtb=" __stringify(CONFIG_TQMA28L_DTB) "\0"			\
	"fdtaddr=0x41000000\0"						\
	"rootblks16=0x20000\0"						\
	"rootpath=/exports/nfsroot\0"					\
	"ipmode=static\0"						\
									\
	"upd_uboot_net=tftp $uboot && "					\
		"setexpr r1 $filesize / 0x200 && setexpr r1 $r1 + 1 && "\
		"mmc dev 1 && mmc write $loadaddr 4001 $r1 && "		\
		"mw 0x80056078 0x2 && "					\
		"echo Copied U-Boot image from ethernet to emmc\0"	\
	"upd_uboot_sd=mmc dev 0 && mmc read $loadaddr 4000 4000 && "	\
		"mmc dev 1 && mmc write $loadaddr 4000 4000 && "	\
		"mw 0x80056078 0x2 && "					\
		"echo Copied U-Boot image from sd card to emmc\0"	\
	"upd_kernel_net=tftp $kernel && "				\
		"setexpr r1 $filesize / 0x200 && setexpr r1 $r1 + 1 && "\
		"mmc dev 1 && mmc write $loadaddr 8000 $r1 && "		\
		"echo Copied Kernel image from ethernet to emmc\0"	\
	"upd_kernel_sd=mmc dev 0 && mmc read $loadaddr 8000 4000 && "	\
		"mmc dev 1 && mmc write $loadaddr 8000 4000 && "	\
		"echo Copied Kernel image from sd card to emmc\0"	\
	"upd_fdt_net=tftp $dtb && "					\
		"setexpr r1 $filesize / 0x200 && setexpr r1 $r1 + 1 && "\
		"mmc dev 1 && mmc write $loadaddr 3000 $r1 && "		\
		"echo Copied device tree blob from ethernet to emmc\0"	\
	"upd_fdt_sd=mmc dev 0 && mmc read $loadaddr 3000 1000 && "	\
		"mmc dev 1 && mmc write $loadaddr 3000 1000 && "	\
		"echo Copied device tree blob from sd card to emmc\0"	\
	"install_firmware="						\
		"echo Downloading MBR... && "				\
		"mmc dev 0 && mmc read $loadaddr 0 1 && "		\
		"mmc dev 1 && mmc write $loadaddr 0 1 && "		\
		"echo Installing device tree blob... && "		\
		"mmc dev 0 && mmc read $loadaddr 3000 1000 && "		\
		"mmc dev 1 && mmc write $loadaddr 3000 1000 && "	\
		"echo Installing U-Boot... && "				\
		"mmc dev 0 && mmc read $loadaddr 4000 4000 && "		\
		"mmc dev 1 && mmc write $loadaddr 4000 4000 && "	\
		"echo Installing Kernel... && "				\
		"mmc dev 0 && mmc read $loadaddr 8000 4000 && "		\
		"mmc dev 1 && mmc write $loadaddr 8000 4000 && "	\
		"echo Installing Root FS (90 MiB chunks) && "		\
		"run root_loop\0"					\
									\
	"root_loop="							\
		"setenv start C000 && "					\
		"setenv r1 $rootblks16 && "				\
		"while itest $r1 -gt 0; do "				\
		  "if itest $r1 -gt 0x2D000; "				\
		    "then setenv count 2D000; "				\
		    "else setenv count $r1; "				\
		  "fi && "						\
		  "mmc dev 0 && mmc read $loadaddr $start $count && "	\
		  "mmc dev 1 && mmc write $loadaddr $start $count && "	\
		  "setexpr start $start + $count && "			\
		  "setexpr r1 $r1 - $count"				\
		"; done; setenv r1; setenv start; setenv count\0"	\
									\
	"addether=setenv bootargs $bootargs fec_mac=$ethaddr\0"		\
	"addip_static=setenv bootargs $bootargs ip=$ipaddr:$serverip:"	\
		"$gatewayip:$netmask:$hostname:$netdev:off\0"		\
	"addip_dyn=setenv bootargs $bootargs ip=dhcp\0"			\
	"addip=if test \"$ipmode\" != static; then "			\
		"run addip_dyn; else run addip_static; fi\0"		\
	"addlcd=setenv bootargs $bootargs lcd_panel=$lcdpanel\0"	\
	"addmisc=setenv bootargs $bootargs ssp1 panic=1\0"		\
	"addmmc=setenv bootargs $bootargs "				\
		"root=/dev/mmcblk${mmcdev}p3 rw rootwait\0"		\
	"addnfs=setenv bootargs $bootargs root=/dev/nfs rw "		\
		"ramdisk_size=$rd_size "				\
		"nfsroot=$serverip:$rootpath,v3,tcp\0"			\
	"addtty=setenv bootargs $bootargs console=$console,$baudrate\0"	\
									\
	"boot_nfs=run addip addnfs addtty addlcd addmisc; "		\
		"tftp $loadaddr $kernel; tftp $fdtaddr $dtb; "		\
		"bootz $loadaddr - $fdtaddr\0"				\
	"boot_ssp=run addmmc addtty addlcd addmisc; "			\
		"mmc read $loadaddr 8000 4000; "			\
		"mmc read $fdtaddr 3000 1000; "				\
		"bootz $loadaddr - $fdtaddr\0"				\
									\
	"erase_env=mw.b $loadaddr 0 512; mmc write $loadaddr 2 1\0"	\
									\
	"n=run boot_nfs\0"						\
	"mmc=mmc dev 1; setenv mmcdev 1; run boot_ssp\0"		\
	"sd=mmc dev 0; setenv mmcdev 0; run boot_ssp\0"			\
	"p=ping $serverip\0"

#define CONFIG_BOOTCOMMAND	"run boot_ssp"

/* The rest of the configuration is shared */
#include <configs/mxs.h>

#endif /* __CONFIGS_TQMA28L_H__ */
