/*
 * am335x_dhcom.h
 *
 * Copyright (C) 2015 DH electronics GmbH - http://www.dh-electronics.com/
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __CONFIG_AM335X_DHCOM_H
#define __CONFIG_AM335X_DHCOM_H

#include <configs/ti_am335x_common.h>

/* Don't override the distro default bootdelay */
#undef CONFIG_BOOTDELAY
#include <config_distro_defaults.h>

#ifndef CONFIG_SPL_BUILD
#ifndef CONFIG_FIT
# define CONFIG_FIT
#endif
# define CONFIG_TIMESTAMP
# define CONFIG_LZO
#endif

#define CONFIG_SYS_BOOTM_LEN            (16 << 20)

#define MACH_TYPE_TIAM335EVM            3589    /* Until the next sync */
#define CONFIG_MACH_TYPE                MACH_TYPE_TIAM335EVM
#define CONFIG_BOARD_LATE_INIT

/* enable DHCOM specific code */
#define CONFIG_DHCOM
#ifdef CONFIG_DHCOM
#include <configs/dhcom_common.h>
#define UBOOT_DH_VERSION "0.4.0.0"     /* DH - Version of U-Boot e.g. 1.4.0.1 */
#endif

/* Clock Defines */
#define V_OSCK                          24000000  /* Clock output from T2 */
#define V_SCLK                          (V_OSCK)

/* Custom script for NOR */
#define CONFIG_SYS_LDSCRIPT             "board/dhelectronics/am335x_dhcom/u-boot.lds"

/* Always 128 KiB env size */
#define CONFIG_ENV_SIZE                 (128 << 10)

/* Memory test*/
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START       0x80010000
#define CONFIG_SYS_MEMTEST_END	       0x80100000

/* Enhance our eMMC support / experience. */
#define CONFIG_HSMMC2_8BIT /* use 8-bit interface */
#define CONFIG_CMD_GPT
#define CONFIG_EFI_PARTITION

#define CONFIG_SYS_DEFAULT_MMC_DEV 0

#ifdef CONFIG_NAND
/* commands for nand and ubi/ubifs */
#define CONFIG_LZO
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define CONFIG_RBTREE
#define CONFIG_CMD_MTDPARTS
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_MTD_UBI_FASTMAP

#define EMMCARGS ""

#define NANDARGS \
        "mtdids=" MTDIDS_DEFAULT "\0" \
        "mtdparts=" MTDPARTS_DEFAULT "\0" \
        "load_settings_bin=ubi part gpmc-nand; ubifsmount ubi0:boot; ubifsload ${loadaddr} ${settings_bin_file}\0" \
        "load_splash=ubifsload ${loadaddr} ${splash_file}\0" \
        "loadbootenv=echo Loading u-boot env file ${bootenv_file}...; ubifsload ${loadaddr} ${bootenv_file};\0" \
        "loadfdt=echo Loading device tree ${fdtfile}...; ubifsload ${fdtaddr} ${fdtfile}\0" \
        "loadimage=echo Loading linux ${bootfile}...; ubifsload ${loadaddr} ${bootfile}\0" \
        "nandloados=setenv set_rootfs setenv rootfs ${rootfs}; run set_rootfs;" \
                "run nandargs; " \
                "if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
                        "if run loadfdt; then " \
                                "bootz ${loadaddr} - ${fdtaddr}; " \
                        "else " \
                                "if test ${boot_fdt} = try; then " \
                                        "bootz; " \
                                "else " \
                                        "echo WARN: Cannot load the DT; " \
                                "fi; " \
                        "fi; " \
                "else " \
                        "bootz; " \
                "fi;\0" \
        "nandargs=setenv bootargs console=${console} " \
                "${optargs} " \
                "${rootfs} " \
                "fbcon=${fbcon} ${optargs} dhcom=${dhcom} " \
                "${backlight} ${tilcdc_panel} SN=${SN}\0" \
        "nandroot=ubi0:rootfs rw ubi.mtd=9,2048\0" \
        "nandrootfstype=ubifs rootwait=1\0" \
        "nandboot=echo Booting from nand ...; " \
                "if run loadbootenv; then " \
                        "echo Loaded environment from ${bootenv_file};" \
                        "run importbootenv;" \
                "fi;" \
                "if run loadimage; then " \
                        "run nandloados;" \
                "fi;\0" \
                "run nandargs; " \
                "nand read ${fdtaddr} u-boot-spl-os; " \
                "nand read ${loadaddr} kernel; " \
                "bootz ${loadaddr} - ${fdtaddr}\0"

#define CONFIG_BOOTCOMMAND \
        "update auto; run nandboot;"
                
#else  /* No NAND -> eMMC*/

#undef CONFIG_MTD_DEVICE
#undef CONFIG_MTD_PARTITIONS
#undef CONFIG_RBTREE
#undef CONFIG_CMD_MTDPARTS
#undef CONFIG_CMD_UBI
#undef CONFIG_CMD_UBIFS

#define EMMCARGS \
        "settings_bin_file=default_settings.bin\0" \
        "splash_file=splash.bmp\0" \
        "load_settings_bin=load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${settings_bin_file}\0" \
        "load_splash=load mmc ${mmcdev}:${mmcpart} ${splashimage} ${splash_file}\0" \
        "loadbootenv=load mmc ${mmcdev} ${loadaddr} ${bootenv_file}\0" \
        "loadimage=load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${bootfile}\0" \
        "loadfdt=load mmc ${mmcdev}:${mmcpart} ${fdtaddr} ${fdtfile}\0" \
        "mmcargs=setenv bootargs console=${console} " \
                "${optargs} " \
                "${rootfs} " \
                "fbcon=${fbcon} ${optargs} dhcom=${dhcom} " \
                "${backlight} ${tilcdc_panel} SN=${SN}\0" \
        "mmcdev=" __stringify(CONFIG_SYS_DEFAULT_MMC_DEV) "\0" \
        "mmcpart=1\0" \
        "mmc_rootfs_part=2\0" \
        "mmcloados=setenv set_rootfs setenv rootfs ${rootfs}; run set_rootfs;" \
                "run mmcargs; " \
                "if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
                        "if run loadfdt; then " \
                                "bootz ${loadaddr} - ${fdtaddr}; " \
                        "else " \
                                "if test ${boot_fdt} = try; then " \
                                        "bootz; " \
                                "else " \
                                        "echo WARN: Cannot load the DT; " \
                                "fi; " \
                        "fi; " \
                "else " \
                        "bootz; " \
                "fi;\0" \
        "mmcboot=mmc dev ${mmcdev}; " \
                "if mmc rescan; then " \
                        "echo SD/MMC found on device ${mmcdev};" \
                        "if run loadbootenv; then " \
                                "echo Loaded environment from ${bootenv_file};" \
                                "run importbootenv;" \
                        "fi;" \
                        "if run loadimage; then " \
                                "run mmcloados;" \
                        "fi;" \
                "fi;\0" \

#define CONFIG_BOOTCOMMAND \
        "update auto;" \
        "run mmcboot;" \
        "setenv mmcdev 1; " \
        "run mmcboot;"

#define NANDARGS ""
#endif /* !CONFIG_NAND */

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define BOOTENV_DEV_LEGACY_MMC(devtypeu, devtypel, instance) \
	"bootcmd_" #devtypel #instance "=" \
	"setenv mmcdev " #instance"; "\
	"setenv bootpart " #instance":2 ; "\
	"run mmcboot\0"

#define BOOTENV_DEV_NAME_LEGACY_MMC(devtypeu, devtypel, instance) \
	#devtypel #instance " "

#define BOOTENV_DEV_NAND(devtypeu, devtypel, instance) \
	"bootcmd_" #devtypel "=" \
	"run nandboot\0"

#define BOOTENV_DEV_NAME_NAND(devtypeu, devtypel, instance) \
	#devtypel #instance " "

#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0) \
	func(LEGACY_MMC, legacy_mmc, 0) \
	func(MMC, mmc, 1) \
	func(LEGACY_MMC, legacy_mmc, 1) \
	func(NAND, nand, 0) \
	func(PXE, pxe, na) \
	func(DHCP, dhcp, na)

#include <config_distro_bootcmd.h>

#ifndef CONFIG_SPL_BUILD
#define CONFIG_EXTRA_ENV_SETTINGS \
        DEFAULT_LINUX_BOOT_ENV \
        "boot_fdt=try\0" \
        "bootfile=zImage\0" \
        "fdtfile=/dtbs/am335x-dheva01.dtb\0" \
        "console=ttyO0,115200n8\0" \
        "optargs=\0" \
        "splashimage=0x80000000\0" \
        "splashpos=m,m\0" \
        "setupdateargs=setenv bootargs " \
                "console=${console} src_intf=${src_intf} src_dev_part=${src_dev_part} dhcom=${dhcom} " \
                "${backlight} ${tilcdc_panel} vt.global_cursor_default=0\0" \
        "load_update_kernel=load ${src_intf} ${src_dev_part} ${loadaddr} zImage_${dhcom}.update; run setupdateargs; bootz ${loadaddr} -\0" \
        "netargs=setenv bootargs console=${console} " \
                "${optargs} " \
                "root=/dev/nfs " \
                "nfsroot=${serverip}:${rootpath},${nfsopts} rw " \
                "ip=dhcp\0" \
        "bootenv_file=uLinuxEnv.txt\0" \
        "importbootenv=echo Importing environment ...; " \
                "env import -t -r $loadaddr $filesize\0" \
        "netboot=echo Booting from network ...; " \
                "setenv autoload no; " \
                "dhcp; " \
                "tftp ${loadaddr} ${bootfile}; " \
                "tftp ${fdtaddr} ${fdtfile}; " \
                "run netargs; " \
                "bootz ${loadaddr} - ${fdtaddr}\0" \
        NANDARGS \
        EMMCARGS
#endif

/* NS16550 Configuration */
#define CONFIG_SYS_NS16550_COM1         0x44e09000      /* Base EVM has UART0 */
#define CONFIG_SYS_NS16550_COM2         0x48022000      /* UART1 */
#define CONFIG_BAUDRATE                 115200

/* I2C */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_OMAP24_I2C_SPEED     100000
#define CONFIG_SYS_OMAP24_I2C_SLAVE     1
#define CONFIG_SYS_I2C_OMAP24XX

#define DISPLAY_ADAPTER_EEPROM_I2C_BUS  2

#define CONFIG_CMD_I2C

/* RTC */
#define CONFIG_CMD_DATE
#define CONFIG_CMD_TIME
#define CONFIG_SYS_RTC_BUS_NUM          0 /* I2C0 */
#define CONFIG_RTC_RV3029
#define CONFIG_SYS_I2C_RTC_ADDR         0x56 /* RTC RV-3029-C3 */

/* PMIC support */
#define CONFIG_POWER_TPS65217

/* SPL */
#ifndef CONFIG_NOR_BOOT
#define CONFIG_SPL_POWER_SUPPORT

/* Bootcount using the RTC block */
#define CONFIG_BOOTCOUNT_LIMIT
#define CONFIG_BOOTCOUNT_AM33XX
#define CONFIG_SYS_BOOTCOUNT_BE

 /* Only interrupt autoboot if <del> is pressed. Otherwise, garbage
  * data on the serial line may interrupt the boot sequence.
  */
#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY                0
#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_AUTOBOOT
#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_AUTOBOOT_PROMPT          \
        "Press DEL to abort autoboot\n"
#define CONFIG_AUTOBOOT_STOP_STR        "\x7f"

/* USB gadget RNDIS */
/* #define CONFIG_SPL_MUSB_NEW_SUPPORT */

#define CONFIG_SPL_LDSCRIPT             "$(CPUDIR)/am33xx/u-boot-spl.lds"
#endif

#ifdef CONFIG_NAND
/* NAND: device related configs */
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_PAGE_COUNT      (CONFIG_SYS_NAND_BLOCK_SIZE / \
                                         CONFIG_SYS_NAND_PAGE_SIZE)
#define CONFIG_SYS_NAND_PAGE_SIZE       2048
#define CONFIG_SYS_NAND_OOBSIZE         64
#define CONFIG_SYS_NAND_BLOCK_SIZE      (128*1024)
#define CONFIG_SYS_NAND_USE_FLASH_BBT
/* NAND: driver related configs */
#define CONFIG_NAND_OMAP_GPMC
#define CONFIG_NAND_OMAP_GPMC_PREFETCH
#define CONFIG_NAND_OMAP_ELM
#define CONFIG_SYS_NAND_BAD_BLOCK_POS   NAND_LARGE_BADBLOCK_POS
#define CONFIG_SYS_NAND_ECCPOS          { 2, 3, 4, 5, 6, 7, 8, 9, \
                                         10, 11, 12, 13, 14, 15, 16, 17, \
                                         18, 19, 20, 21, 22, 23, 24, 25, \
                                         26, 27, 28, 29, 30, 31, 32, 33, \
                                         34, 35, 36, 37, 38, 39, 40, 41, \
                                         42, 43, 44, 45, 46, 47, 48, 49, \
                                         50, 51, 52, 53, 54, 55, 56, 57, }

#define CONFIG_SYS_NAND_ECCSIZE         512
#define CONFIG_SYS_NAND_ECCBYTES        14
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_NAND_OMAP_ECCSCHEME      OMAP_ECC_BCH8_CODE_HW
#define MTDIDS_DEFAULT                  "nand0=nand.0"
#define MTDPARTS_DEFAULT                "mtdparts=nand.0:" \
                                        "-(gpmc-nand)"

/* NAND: SPL related configs */
#undef CONFIG_SPL_NAND_SUPPORT /* spl needs no nand support (uboot is in spi nor-flash )*/
#ifdef CONFIG_SPL_NAND_SUPPORT
#define CONFIG_SPL_NAND_AM33XX_BCH
#endif
#undef CONFIG_SPL_OS_BOOT /* dhcom: use bootloader u-boot for os boot (not spl) */
#ifdef CONFIG_SPL_OS_BOOT
#define CONFIG_CMD_SPL_NAND_OFS 0x00080000 /* os parameters */
#define CONFIG_SYS_NAND_SPL_KERNEL_OFFS 0x00200000 /* kernel offset */
#define CONFIG_CMD_SPL_WRITE_SIZE       0x2000
#endif
#endif /* !CONFIG_NAND */

/*
 * For NOR boot, we must set this to the start of where NOR is mapped
 * in memory.
 */
#ifdef CONFIG_NOR_BOOT
#define CONFIG_SYS_TEXT_BASE            0x08000000
#endif

/*
 * USB configuration.  We enable MUSB support, both for host and for
 * gadget.  We set USB0 as peripheral and USB1 as host, based on the
 * board schematic and physical port wired to each.  Then for host we
 * add mass storage support and for gadget we add both RNDIS ethernet
 * and DFU.
 */
#define CONFIG_USB_MUSB_DSPS
#define CONFIG_ARCH_MISC_INIT
#define CONFIG_USB_MUSB_GADGET
#define CONFIG_USB_MUSB_PIO_ONLY
#define CONFIG_USB_MUSB_DISABLE_BULK_COMBINE_SPLIT
#define CONFIG_USB_GADGET
#define CONFIG_USB_GADGET_DOWNLOAD
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_GADGET_VBUS_DRAW     2
#define CONFIG_USB_MUSB_HOST
#define CONFIG_AM335X_USB0
#define CONFIG_AM335X_USB0_MODE MUSB_PERIPHERAL
#define CONFIG_AM335X_USB1
#define CONFIG_AM335X_USB1_MODE MUSB_HOST

#ifndef CONFIG_SPL_USBETH_SUPPORT
/* Fastboot */
#define CONFIG_CMD_FASTBOOT
#define CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_USB_FASTBOOT_BUF_ADDR    CONFIG_SYS_LOAD_ADDR
#define CONFIG_USB_FASTBOOT_BUF_SIZE    0x07000000

/* To support eMMC booting */
#define CONFIG_STORAGE_EMMC
#define CONFIG_FASTBOOT_FLASH_MMC_DEV   CONFIG_SYS_DEFAULT_MMC_DEV
#endif

#ifdef CONFIG_USB_MUSB_HOST
#define CONFIG_CMD_USB
#define CONFIG_USB_STORAGE
#endif

#ifdef CONFIG_USB_MUSB_GADGET
/* Removing USB gadget and can be enabled adter adding support usb DM */
#ifndef CONFIG_DM_ETH
#define CONFIG_USB_ETHER
#define CONFIG_USB_ETH_RNDIS
#define CONFIG_USBNET_HOST_ADDR "de:ad:be:af:00:00"
#endif /* CONFIG_DM_ETH */

/* USB TI's IDs */
#define CONFIG_G_DNL_VENDOR_NUM 0x0451
#define CONFIG_G_DNL_PRODUCT_NUM 0xD022
#define CONFIG_G_DNL_MANUFACTURER "Texas Instruments"
#endif /* CONFIG_USB_MUSB_GADGET */

/*
 * Disable MMC DM for SPL build and can be re-enabled after adding
 * DM support in SPL
 */
#ifdef CONFIG_SPL_BUILD
#undef CONFIG_DM_MMC
#endif

/* Remove other SPL modes. */
#undef CONFIG_SPL_YMODEM_SUPPORT
#undef CONFIG_SPL_NAND_SUPPORT
#undef CONFIG_SPL_MMC_SUPPORT

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_USBETH_SUPPORT)
#define CONFIG_ENV_IS_NOWHERE
#undef CONFIG_ENV_IS_IN_NAND
/* disable host part of MUSB in SPL */
#undef CONFIG_MUSB_HOST
/* disable EFI partitions and partition UUID support */
#undef CONFIG_PARTITION_UUIDS
#undef CONFIG_EFI_PARTITION
/* General network SPL  */
#define CONFIG_SPL_NET_SUPPORT
#define CONFIG_SPL_ENV_SUPPORT
#define CONFIG_SPL_NET_VCI_STRING       "AM335x U-Boot SPL"
#endif

/*
 * Default to using SPI for environment, etc.
 * 0x000000 - 0x020000 : SPL (128KiB)
 * 0x020000 - 0x0A0000 : U-Boot (512KiB)
 * 0x0A0000 - 0x0BFFFF : First copy of U-Boot Environment (128KiB)
 * 0x0C0000 - 0x0DFFFF : Second copy of U-Boot Environment (128KiB)
 * 0x0E0000 - 0x442000 : Linux Kernel
 * 0x442000 - 0x800000 : Userland
 */
#if defined(CONFIG_SPI_BOOT)
/* SPL related */
#undef CONFIG_SPL_OS_BOOT		/* Not supported by existing map */
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x20000

#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#define CONFIG_ENV_SECT_SIZE		(4 << 10) /* 4 KB sectors */
#define CONFIG_ENV_OFFSET		(768 << 10) /* 768 KiB in */
#define CONFIG_ENV_OFFSET_REDUND	(896 << 10) /* 896 KiB in */
#define MTDIDS_DEFAULT_NOR              "nor0=m25p80-flash.0"
#define MTDPARTS_DEFAULT_NOR            "mtdparts=m25p80-flash.0:128k(SPL)," \
                                        "512k(u-boot),128k(u-boot-env1)," \
                                        "128k(u-boot-env2),-(blank)"
#endif

/* SPI flash. */
#define CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_SPEED         48000000

/* Network. - On DHCOM AM335x we have RMII */
#undef CONFIG_MII
#define CONFIG_RMII
#define CONFIG_PHY_GIGE
#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC

/* LCD support */
#define CONFIG_AM335X_LCD
#define CONFIG_LCD
#define CONFIG_LCD_LOGO
#define CONFIG_LCD_NOSTDOUT
#define CONFIG_SYS_BLACK_ON_WHITE
#define LCD_BPP LCD_COLOR32

#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN

#define CONFIG_VIDEO_BMP_GZIP
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE  (1366*767*4)
#define CONFIG_CMD_UNZIP
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_BMP_24BMP
#define CONFIG_BMP_32BPP

#endif  /* ! __CONFIG_AM335X_DHCOM_H */
