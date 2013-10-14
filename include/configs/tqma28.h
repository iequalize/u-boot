/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
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
#ifndef __TQMA28_H
#define __TQMA28_H

#if !defined(__ASSEMBLY__)
extern unsigned tqma28_get_mmc_devid(void);
#endif

#define CONFIG_TQMA28
#define CONFIG_MBA28
/*
 * SoC configurations
 */
#define CONFIG_MX28				/* i.MX28 SoC */
#define CONFIG_MXS_GPIO			/* GPIO control */
#define CONFIG_SYS_HZ		1000		/* Ticks per second */

/* This should be removed after it's added into mach-types.h */
#ifndef MACH_TYPE_TQMA28
#define MACH_TYPE_TQMA28	4223
#endif

#define CONFIG_MACH_TYPE	MACH_TYPE_TQMA28

#include <asm/arch/regs-base.h>

#define CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_ICACHE_OFF
#define CONFIG_SYS_DCACHE_OFF
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_ARCH_MISC_INIT

/*
 * SPL
 */
#define CONFIG_SPL
#define CONFIG_SPL_NO_CPU_SUPPORT_CODE
#define CONFIG_SPL_START_S_PATH	"arch/arm/cpu/arm926ejs/mxs"
#define CONFIG_SPL_LDSCRIPT	"arch/arm/cpu/arm926ejs/mxs/u-boot-spl.lds"
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_GPIO_SUPPORT
#define CONFIG_SPL_MXS_NO_DCDC_BATT_SOURCE

/*
 * U-Boot Commands
 */
#include <config_cmd_default.h>
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DOS_PARTITION
#define CONFIG_MISC_INIT_R

#define CONFIG_CMD_CACHE
#define CONFIG_CMD_DATE
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_GPIO
#define CONFIG_CMD_MII
#define CONFIG_CMD_MMC
#define CONFIG_CMD_NET
#define CONFIG_CMD_NFS
#define CONFIG_CMD_PING
#if 0
#define CONFIG_CMD_SPI /* cmd_spi uses full-duplex, not supported by mx28 */
#endif
#if 0
#define CONFIG_CMD_USB /* disable USB support for minimum interface policy */
#endif
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_I2C
#define CONFIG_CMD_EEPROM
#define CONFIG_CMD_DTT
#define CONFIG_CMD_SETEXPR
#define CONFIG_CMD_ITEST

/*
 * Memory configurations
 */
#define CONFIG_NR_DRAM_BANKS		1		/* 1 bank of DRAM */
#define PHYS_SDRAM_1			0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE		0x40000000	/* Max 1 GB RAM */
#define CONFIG_STACKSIZE		(128 * 1024)	/* 128 KB stack */
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
/*
 * U-Boot general configurations
 */
#define CONFIG_SYS_LONGHELP
/* TODO: implement following definition */
#ifdef CONFIG_TQMA28_RELEASE_REVISION
#define CONFIG_SYS_PROMPT	("TQMa28 " \
					MK_STR(CONFIG_TQMA28_RELEASE_REVISION) \
					" U-Boot > ")
#else
#define CONFIG_SYS_PROMPT	"TQMa28 U-Boot > "
#endif
#define CONFIG_SYS_CBSIZE	1024		/* Console I/O buffer size */
#define CONFIG_SYS_PBSIZE	\
	(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
						/* Print buffer size */
#define CONFIG_SYS_MAXARGS	32		/* Max number of command args */
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE
						/* Boot argument buffer size */
#define CONFIG_VERSION_VARIABLE	/* U-BOOT version */
#define CONFIG_AUTO_COMPLETE		/* Command auto complete */
#define CONFIG_CMDLINE_EDITING		/* Command history etc */
#define CONFIG_SYS_HUSH_PARSER

/*
 * Serial Driver
 */

#define CONFIG_MXS_UARTAPP
#define CONFIG_UARTAPP_CLK		24000000
#define MXS_UARTAPP_BASE		MXS_UARTAPP3_BASE
#define CONFIG_BAUDRATE			115200	/* Default baud rate */
#define CONFIG_SYS_LINUX_CONSOLE	ttyAPP3

/*
 * DMA
 */
#define CONFIG_APBH_DMA

/*
 * MMC Driver
 */
#define CONFIG_ENV_IS_IN_MMC
#ifdef CONFIG_ENV_IS_IN_MMC
 #define CONFIG_ENV_OFFSET	(0x400) /* 1 KB */
 #define CONFIG_ENV_SIZE	(0x20000 - 0x400) /* 127 KB */
 #define CONFIG_SYS_MMC_ENV_DEV	tqma28_get_mmc_devid()
#endif
#define CONFIG_CMD_SAVEENV
#ifdef	CONFIG_CMD_MMC
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_MXS_MMC
#endif

/*
 * Ethernet on SOC (FEC)
 */
#ifdef	CONFIG_CMD_NET
#define CONFIG_NET_MULTI
#define CONFIG_ETHPRIME	"FEC0"
#define CONFIG_FEC_MXC
#define CONFIG_FEC_MXC_MULTI
#define CONFIG_MII
#define CONFIG_DISCOVER_PHY
#define CONFIG_FEC_XCV_TYPE	RMII
#define CONFIG_MX28_FEC_MAC_IN_OCOTP
#endif

/*
 * RTC
 */
#ifdef	CONFIG_CMD_DATE
#define	CONFIG_RTC_MXS
#endif

/*
 * USB
 */
#ifdef	CONFIG_CMD_USB
#define	CONFIG_USB_EHCI
#define	CONFIG_USB_EHCI_MXS
#define	CONFIG_EHCI_MXS_PORT 1
#define	CONFIG_EHCI_IS_TDI
#define	CONFIG_USB_STORAGE
#endif

/* I2C */
#ifdef CONFIG_CMD_I2C
#define CONFIG_I2C_MXS
#define CONFIG_HARD_I2C
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
#define CONFIG_HARD_SPI
#define CONFIG_MXS_SPI
#define CONFIG_SPI_HALF_DUPLEX
#define CONFIG_DEFAULT_SPI_BUS		2
#define CONFIG_DEFAULT_SPI_MODE		SPI_MODE_0
#endif

/*
 * Boot Linux
 */
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_BOOTDELAY	3
#define CONFIG_BOOTFILE	"zImage"
#define CONFIG_BOOTCOMMAND	"run boot_ssp"
#define CONFIG_LOADADDR	0x42000000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR
#define CONFIG_OF_LIBFDT

/*
 * Extra Environments
 *
 * TODO: We still have to find out the amount of sectors consumed
 * by the U-Boot image on sd card updates, so we can write the redundant copy
 * behind the first. This number must be updated also in the MBR.
 */
#define	CONFIG_EXTRA_ENV_SETTINGS \
	"rd_size=16384\0"						\
	"netdev=eth0\0"							\
	"console=" __stringify(CONFIG_SYS_LINUX_CONSOLE) "\0"		\
	"lcdpanel=fg0700\0"						\
	"kernel=zImage\0"						\
	"uboot=u-boot.sb\0"						\
	"dtb=imx28-mba28.dtb\0"						\
	"fdtaddr=0x41000000\0"						\
	"rootblks16=0x20000\0"						\
	"rootpath=/exports/nfsroot\0"					\
	"ipmode=static\0"						\
									\
	"upd_uboot_net=tftp $uboot && "					\
		"setexpr r1 $filesize / 0x200 && setexpr r1 $r1 + 1 && "\
		"mmc dev 0 && mmc write $loadaddr 4001 $r1 && "		\
		"mw 0x80056078 0x2 && "					\
		"echo Copied U-Boot image from ethernet to emmc\0"	\
	"upd_uboot_sd=mmc dev 1 && mmc read $loadaddr 4000 4000 && "	\
		"mmc dev 0 && mmc write $loadaddr 4000 4000 && "	\
		"mw 0x80056078 0x2 && "					\
		"echo Copied U-Boot image from sd card to emmc\0"	\
	"upd_kernel_net=tftp $kernel && "				\
		"setexpr r1 $filesize / 0x200 && setexpr r1 $r1 + 1 && "\
		"mmc dev 0 && mmc write $loadaddr 8000 $r1 && "		\
		"echo Copied Kernel image from ethernet to emmc\0"	\
	"upd_kernel_sd=mmc dev 1 && mmc read $loadaddr 8000 4000 && "	\
		"mmc dev 0 && mmc write $loadaddr 8000 4000 && "	\
		"echo Copied Kernel image from sd card to emmc\0"	\
	"upd_fdt_net=tftp $dtb && "					\
		"setexpr r1 $filesize / 0x200 && setexpr r1 $r1 + 1 && "\
		"mmc dev 0 && mmc write $loadaddr 3000 $r1 && "		\
		"echo Copied device tree blob from ethernet to emmc\0"	\
	"upd_fdt_sd=mmc dev 1 && mmc read $loadaddr 3000 1000 && "	\
		"mmc dev 0 && mmc write $loadaddr 3000 1000 && "	\
		"echo Copied device tree blob from sd card to emmc\0"	\
	"install_firmware="						\
		"echo Downloading MBR... && "				\
		"mmc dev 1 && mmc read $loadaddr 0 1 && "		\
		"mmc dev 0 && mmc write $loadaddr 0 1 && "		\
		"echo Installing device tree blob... && "		\
		"mmc dev 1 && mmc read $loadaddr 3000 1000 && "		\
		"mmc dev 0 && mmc write $loadaddr 3000 1000 && "	\
		"echo Installing U-Boot... && "				\
		"mmc dev 1 && mmc read $loadaddr 4000 4000 && "		\
		"mmc dev 0 && mmc write $loadaddr 4000 4000 && "	\
		"echo Installing Kernel... && "				\
		"mmc dev 1 && mmc read $loadaddr 8000 4000 && "		\
		"mmc dev 0 && mmc write $loadaddr 8000 4000 && "	\
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
		  "mmc dev 1 && mmc read $loadaddr $start $count && "	\
		  "mmc dev 0 && mmc write $loadaddr $start $count && "	\
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
	"boot_nfs=run addether addip addnfs addtty addlcd addmisc; "	\
		"tftp $loadaddr $kernel; tftp $fdtaddr $dtb; "		\
		"bootz $loadaddr - $fdtaddr\0"				\
	"boot_ssp=run addether addmmc addtty addlcd addmisc; "	\
		"mmc read $loadaddr 8000 4000; "			\
		"mmc read $fdtaddr 3000 1000; "				\
		"bootz $loadaddr - $fdtaddr\0"				\
									\
	"erase_env=mw.b $loadaddr 0 512; mmc write $loadaddr 2 1\0"	\
									\
	"ipaddr=172.168.135.170\0"					\
	"serverip=172.168.135.107\0"					\
	"netmask=255.255.255.0\0"					\
	"ethaddr=02:00:01:00:00:01\0"					\
	"n=run boot_nfs\0"						\
	"mmc=mmc dev 0; setenv mmcdev 0; run boot_ssp\0"		\
	"sd=mmc dev 1; setenv mmcdev 1; run boot_ssp\0"			\
	"p=ping $serverip\0"

/* The global boot mode will be detected by ROM code and
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

#endif /* __TQMA28_H */
