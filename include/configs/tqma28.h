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
#ifndef __CONFIGS_TQMA28_H__
#define __CONFIGS_TQMA28_H__

#if !defined(__ASSEMBLY__)
extern unsigned tqma28_get_env_dev(void);
#endif

/* System configurations */
#define CONFIG_MX28				/* i.MX28 SoC */
#define CONFIG_MACH_TYPE	MACH_TYPE_TQMA28

#include <asm/arch/regs-base.h>
#include <linux/sizes.h>

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
#define CONFIG_CMD_DTT
#define CONFIG_CMD_EEPROM
#define CONFIG_CMD_FAT
#define CONFIG_CMD_FUSE
#define CONFIG_CMD_GPIO
#define CONFIG_CMD_I2C
#define CONFIG_CMD_MII
#define CONFIG_CMD_MMC
#define CONFIG_CMD_MTDPARTS
#define CONFIG_CMD_PING
#define CONFIG_CMD_SF
#if 0
#define CONFIG_CMD_SPI /* cmd_spi uses full-duplex, not supported by mx28 */
#endif
#define CONFIG_CMD_USB

/* Memory configuration */
#define CONFIG_NR_DRAM_BANKS		1		/* 1 bank of DRAM */
#define PHYS_SDRAM_1			0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE		0x40000000	/* Max 1 GB RAM */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1
#define CONFIG_STACKSIZE		(128 * 1024)	/* 128 KB stack */

/* Environment */
#if defined(CONFIG_CMD_MMC)
#define CONFIG_SD_INDEX 1
#define CONFIG_MMC_INDEX 0
#endif
/* Environment is in MMC */
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_OFFSET	(0x400) /* 1 KB */
#define CONFIG_ENV_SIZE	(0x20000 - 0x400) /* 127 KB */
#endif
#define CONFIG_SYS_MMC_ENV_DEV	tqma28_get_env_dev()

/* SPI Flash */
#ifdef CONFIG_CMD_SF
#define CONFIG_MXS_SPI
/* this may vary and depends on the installed chip */
#define CONFIG_SPI_FLASH_MTD
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SF_DEFAULT_BUS		2
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_3
#define CONFIG_SF_DEFAULT_SPEED		24000000
#define MTDIDS_DEFAULT \
	"nor0=nor0\0"                                                          \

#define MTDPARTS_DEFAULT \
	"mtdparts=nor0:"                                                       \
		"512k@0k(U-Boot-SPI),"                                         \
		 "64k@512k(ENV)\0"
#endif

#if defined(CONFIG_CMD_SF) && defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_SPL_MXS_SPI
#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_OFFSET		SZ_512K
#define CONFIG_ENV_SIZE			SZ_64K
#define CONFIG_ENV_SECT_SIZE		0x10000
#define CONFIG_ENV_SPI_CS		0
#define CONFIG_ENV_SPI_BUS		2
#define CONFIG_ENV_SPI_MAX_HZ		24000000
#define CONFIG_ENV_SPI_MODE		SPI_MODE_3
#endif

#define CONFIG_SYS_PROMPT	"TQMa28 U-Boot > "
#define CONFIG_SYS_PBSIZE	\
	(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
						/* Print buffer size */

/* Serial Driver */
#define CONFIG_SYS_LINUX_CONSOLE	ttyAPP3

/* FEC Ethernet on SoC */
#ifdef	CONFIG_CMD_NET
#define CONFIG_NET_MULTI
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
#endif

/* I2C */
#ifdef CONFIG_CMD_I2C
#define CONFIG_MXS_I2C_BASE	MXS_I2C1_BASE
#define CONFIG_SYS_I2C_SPEED	100000
#endif

/* EEPROM */
#define CONFIG_SYS_I2C_EEPROM_ADDR		0x50
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS	4
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN		2
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	5

/* DTT */
#define CONFIG_DTT_LM73		1
#define CONFIG_DTT_SENSORS	{1}
#define CONFIG_SYS_DTT_BUS_NUM	1
#define CONFIG_SYS_DTT_MAX_TEMP	200
#define CONFIG_SYS_DTT_MIN_TEMP	-100

/*
 * SPI - cmd_spi uses full-duplex, not supported by mx28
 */
#ifdef CONFIG_CMD_SPI
#define CONFIG_DEFAULT_SPI_BUS		2
#define CONFIG_DEFAULT_SPI_MODE		SPI_MODE_0
#endif

/* Boot Linux */
#define CONFIG_BOOTDELAY	1
#define CONFIG_BOOTFILE	"zImage"
#define CONFIG_BOOTCOMMAND	"run boot_ssp"
#define CONFIG_LOADADDR	0x42000000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR
#define CONFIG_SYS_MXS_VDD5V_ONLY
#define CONFIG_OF_BOARD_SETUP
#define CONFIG_FIT

/*
 * Extra Environments
 *
 * TODO: We still have to find out the amount of sectors consumed
 * by the U-Boot image on sd card updates, so we can write the redundant copy
 * behind the first. This number must be updated also in the MBR.
 */
#define TQMA28_FDT_SECT		0x3000
#define TQMA28_FDT_SZ		0x1000
#define TQMA28_UBOOT_SECT	0x4000
#define TQMA28_UBOOT_SZ		0x4000
#define TQMA28_KERNEL_SECT	0x8000
#define TQMA28_KERNEL_SZ	0x4000
#define TQMA28_ROOTFS_SECT	0xC000
#define TQMA28_ROOTFS_SZ	0x20000

#define	CONFIG_EXTRA_ENV_SETTINGS \
	"rd_size=16384\0"										\
	"netdev=eth0\0"											\
	"console=" __stringify(CONFIG_SYS_LINUX_CONSOLE) "\0"						\
	"lcdpanel=fg0700\0"										\
	"kernel=zImage\0"										\
	"uboot=u-boot.sb\0"										\
	"uboot_spi=u-boot_spi.sb\0"									\
	"dtb=imx28-mba28.dtb\0"										\
	"fdtaddr=0x41000000\0"										\
	"rootblks16=" __stringify(TQMA28_ROOTFS_SZ) "\0"						\
	"rootpath=/exports/nfsroot\0"									\
	"ipmode=static\0"										\
													\
	"upd_uboot_spi_net=tftp $uboot_spi && "								\
		"sf probe && sf update $loadaddr U-Boot-SPI $filesize && "				\
		"echo Copied U-Boot image from ethernet to SPI flash\0"					\
	"upd_uboot_net=tftp $uboot && "									\
		"setexpr r1 $filesize / 0x200 && setexpr r1 $r1 + 1 && "				\
		"setexpr r0 " __stringify(TQMA28_UBOOT_SECT) " + 1 && mmc dev 0 && "			\
		"mmc write $loadaddr $r0 $r1 && "							\
		"mw 0x80056078 0x2 && "									\
		"echo Copied U-Boot image from ethernet to emmc\0"					\
	"upd_uboot_sd="											\
		"mmc dev 1 && "										\
		"mmc read $loadaddr " __stringify(TQMA28_UBOOT_SECT) " " __stringify(TQMA28_UBOOT_SZ) " && "\
		"mmc dev 0 && "										\
		"mmc write $loadaddr " __stringify(TQMA28_UBOOT_SECT) " " __stringify(TQMA28_UBOOT_SZ) " && "\
		"mw 0x80056078 0x2 && "									\
		"echo Copied U-Boot image from sd card to emmc\0"					\
	"upd_kernel_net=tftp $kernel && "								\
		"setexpr r1 $filesize / 0x200 && setexpr r1 $r1 + 1 && "				\
		"mmc dev 0 && mmc write $loadaddr " __stringify(TQMA28_KERNEL_SECT) " $r1 && "		\
		"echo Copied Kernel image from ethernet to emmc\0"					\
	"upd_kernel_sd=mmc dev 1 && "									\
		"mmc read $loadaddr " __stringify(TQMA28_KERNEL_SECT) " " __stringify(TQMA28_KERNEL_SZ) " && "\
		"mmc dev 0 && "										\
		"mmc write $loadaddr " __stringify(TQMA28_KERNEL_SECT) " " __stringify(TQMA28_KERNEL_SZ) " && "\
		"echo Copied Kernel image from sd card to emmc\0"					\
	"upd_fdt_net=tftp $dtb && "									\
		"setexpr r1 $filesize / 0x200 && setexpr r1 $r1 + 1 && "				\
		"mmc dev 0 && "										\
		"mmc write $loadaddr " __stringify(TQMA28_FDT_SECT) " $r1 && "				\
		"echo Copied device tree blob from ethernet to emmc\0"					\
	"upd_fdt_sd=mmc dev 1 && "									\
		"mmc read $loadaddr " __stringify(TQMA28_FDT_SECT) " " __stringify(TQMA28_FDT_SZ) " && "\
		"mmc dev 0 && "										\
		"mmc write $loadaddr " __stringify(TQMA28_FDT_SECT) " " __stringify(TQMA28_FDT_SZ) " && "\
		"echo Copied device tree blob from sd card to emmc\0"					\
	"install_firmware="										\
		"echo Downloading MBR... && "								\
		"mmc dev 1 && mmc read $loadaddr 0 1 && "						\
		"mmc dev 0 && mmc write $loadaddr 0 1 && "						\
		"echo Installing device tree blob... && "						\
		"mmc dev 1 && "										\
		"mmc read $loadaddr " __stringify(TQMA28_FDT_SECT) " " __stringify(TQMA28_FDT_SZ) " && "\
		"mmc dev 0 && "										\
		"mmc write $loadaddr " __stringify(TQMA28_FDT_SECT) " " __stringify(TQMA28_FDT_SZ) " && "\
		"echo Installing U-Boot... && "								\
		"mmc dev 1 && "										\
		"mmc read $loadaddr " __stringify(TQMA28_UBOOT_SECT) " " __stringify(TQMA28_UBOOT_SZ) " && "\
		"mmc dev 0 && "										\
		"mmc write $loadaddr " __stringify(TQMA28_UBOOT_SECT) " " __stringify(TQMA28_UBOOT_SZ) " && "\
		"echo Installing Kernel... && "								\
		"mmc dev 1 && "										\
		"mmc read $loadaddr " __stringify(TQMA28_KERNEL_SECT) " " __stringify(TQMA28_KERNEL_SZ) " && "\
		"mmc dev 0 && "										\
		"mmc write $loadaddr " __stringify(TQMA28_KERNEL_SECT) " " __stringify(TQMA28_KERNEL_SZ) " && "\
		"echo Installing Root FS (90 MiB chunks) && "						\
		"run root_loop\0"									\
													\
	"root_loop="											\
		"setenv start " __stringify(TQMA28_ROOTFS_SECT) " && "					\
		"setenv r1 $rootblks16 && "								\
		"while itest $r1 -gt 0; do "								\
		  "if itest $r1 -gt 0x2D000; "								\
		    "then setenv count 2D000; "								\
		    "else setenv count $r1; "								\
		  "fi && "										\
		  "mmc dev 1 && mmc read $loadaddr $start $count && "					\
		  "mmc dev 0 && mmc write $loadaddr $start $count && "					\
		  "setexpr start $start + $count && "							\
		  "setexpr r1 $r1 - $count"								\
		"; done; setenv r1; setenv start; setenv count\0"					\
													\
	"addip_static=setenv bootargs $bootargs ip=$ipaddr:$serverip:"					\
		"$gatewayip:$netmask:$hostname:$netdev:off\0"						\
	"addip_dyn=setenv bootargs $bootargs ip=dhcp\0"							\
	"addip=if test \"$ipmode\" != static; then "							\
		"run addip_dyn; else run addip_static; fi\0"						\
	"addlcd=setenv bootargs $bootargs lcd_panel=$lcdpanel\0"					\
	"addmisc=setenv bootargs $bootargs ssp1 panic=1\0"						\
	"addmmc=setenv bootargs $bootargs "								\
		"root=/dev/mmcblk${mmcdev}p3 rw rootwait\0"						\
	"addnfs=setenv bootargs $bootargs root=/dev/nfs rw "						\
		"ramdisk_size=$rd_size "								\
		"nfsroot=$serverip:$rootpath,v3,tcp\0"							\
	"addtty=setenv bootargs $bootargs "								\
		"console=tty0 consoleblank=0 "								\
		"console=$console,$baudrate\0"								\
													\
	"boot_nfs=run addip addnfs addtty addlcd addmisc; "						\
		"tftp $loadaddr $kernel; tftp $fdtaddr $dtb; "						\
		"bootz $loadaddr - $fdtaddr\0"								\
	"boot_ssp=run addmmc addtty addlcd addmisc; "							\
		"mmc read $loadaddr " __stringify(TQMA28_KERNEL_SECT) " " __stringify(TQMA28_KERNEL_SZ) "; "	\
		"mmc read $fdtaddr " __stringify(TQMA28_FDT_SECT) " " __stringify(TQMA28_FDT_SZ) "; "		\
		"bootz $loadaddr - $fdtaddr\0"								\
													\
	ERASEENV											\
	"n=run boot_nfs\0"										\
	"mmc=mmc dev 0; setenv mmcdev 0; run boot_ssp\0"						\
	"sd=mmc dev 1; setenv mmcdev 1; run boot_ssp\0"							\
	"p=ping $serverip\0"

#if defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define ERASEENV	"erase_env=sf probe && sf erase ENV 0x10000\0"
#else
#define ERASEENV	"erase_env=mw.b $loadaddr 0 512; mmc write $loadaddr 2 1\0"
#endif

#define BOOT_MODE_SD0 0x9
#define BOOT_MODE_SD1 0xa

/* The rest of the configuration is shared */
#include <configs/mxs.h>

#endif /* __TQMA28_H */
