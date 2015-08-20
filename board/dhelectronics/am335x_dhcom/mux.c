/*
 * mux.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <i2c.h>
#include "board.h"

static struct module_pin_mux uart0_pin_mux[] = { /* UART1 on DHCOM */
	{OFFSET(uart0_rxd), (MODE(0) | PULLUP_EN | RXACTIVE)},	/* UART0_RXD */
	{OFFSET(uart0_txd), (MODE(0) | PULLUDEN)},		/* UART0_TXD */
	{OFFSET(uart0_ctsn), (MODE(0) | PULLUP_EN | RXACTIVE)},	/* UART0_CTS */
	{OFFSET(uart0_rtsn), (MODE(0) | PULLUDEN)},		/* UART0_RTS */
	{-1},
};

static struct module_pin_mux uart1_pin_mux[] = { /* UART2 on DHCOM */
	{OFFSET(uart1_rxd), (MODE(0) | PULLUP_EN | RXACTIVE)},	/* UART1_RXD */
	{OFFSET(uart1_txd), (MODE(0) | PULLUDEN)},		/* UART1_TXD */
	{-1},
};

static struct module_pin_mux mmc0_pin_mux[] = { /* SD/µSD Interface on DHCOM */
	{OFFSET(mmc0_dat3), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_DAT3 */
	{OFFSET(mmc0_dat2), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_DAT2 */
	{OFFSET(mmc0_dat1), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_DAT1 */
	{OFFSET(mmc0_dat0), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_DAT0 */
	{OFFSET(mmc0_clk), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_CLK */
	{OFFSET(mmc0_cmd), (MODE(0) | RXACTIVE | PULLUP_EN)},	/* MMC0_CMD */
	{OFFSET(emu1), (MODE(7) | RXACTIVE  | PULLUDDIS)},	/* MMC0_CD */
	{-1},
};

static struct module_pin_mux i2c0_pin_mux[] = { /* on module I2C at DHCOM */
	{OFFSET(i2c0_sda), (MODE(0) | RXACTIVE |
			PULLUDEN | SLEWCTRL)}, /* I2C_DATA */
	{OFFSET(i2c0_scl), (MODE(0) | RXACTIVE |
			PULLUDEN | SLEWCTRL)}, /* I2C_SCLK */
	{-1},
};

static struct module_pin_mux i2c2_pin_mux[] = { /* I2C1 on DHCOM */
	{OFFSET(uart1_ctsn), (MODE(3) | RXACTIVE |
			PULLUDEN | SLEWCTRL)}, /* I2C_DATA */
	{OFFSET(uart1_rtsn), (MODE(3) | RXACTIVE |
			PULLUDEN | SLEWCTRL)}, /* I2C_SCLK */
	{-1},
};

static struct module_pin_mux spi0_pin_mux[] = { /* Bootloader NOR-Flash + DHCOM SPI1 */
	{OFFSET(spi0_sclk), (MODE(0) | RXACTIVE | PULLUDEN)},	/* SPI0_SCLK */
	{OFFSET(spi0_d0), (MODE(0) | RXACTIVE |
			PULLUDEN | PULLUP_EN)},			/* SPI0_D0 */
	{OFFSET(spi0_d1), (MODE(0) | RXACTIVE | PULLUDEN)},	/* SPI0_D1 */
	{OFFSET(spi0_cs0), (MODE(0) | RXACTIVE |
			PULLUDEN | PULLUP_EN)},			/* SPI0_CS0 : Nor-Flash */
	{OFFSET(spi0_cs1), (MODE(0) | RXACTIVE |
			PULLUDEN | PULLUP_EN)},			/* SPI0_CS1 : DHCOM SPI1 */
	{-1},
};

#if defined(CONFIG_DHCOM_USE_SPI2)
static struct module_pin_mux spi1_pin_mux[] = { /* Only optional: DHCOM SPI2 */
	{OFFSET(ecap0_in_pwm0_out), (MODE(4) | RXACTIVE | PULLUDEN)},	/* SPI1_SCLK */
	{OFFSET(mcasp0_fsx), (MODE(3) | RXACTIVE |
			PULLUDEN | PULLUP_EN)},			/* SPI1_D0 */
	{OFFSET(mcasp0_axr0), (MODE(3) | RXACTIVE | PULLUDEN)},	/* SPI1_D1 */
	{OFFSET(mcasp0_ahclkr), (MODE(3) | RXACTIVE |
			PULLUDEN | PULLUP_EN)},			/* SPI1_CS0 : DHCOM SPI2 */
	{-1},
};
#endif

static struct module_pin_mux dhcom_gpio1_pin_mux[] = { /* DHCOM GPIOs - A:E */
	{OFFSET(xdma_event_intr1), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO A , Port 0 Pin 20 */
	{OFFSET(mcasp0_fsr), (MODE(7) | PULLUDDIS | RXACTIVE)},	        /* GPIO B , Port 3 Pin 19 */
	{OFFSET(mcasp0_axr1), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO C , Port 3 Pin 20 */
	{OFFSET(gpmc_clk), (MODE(7) | PULLUDDIS | RXACTIVE)},	        /* GPIO D , Port 2 Pin 1 */
	{OFFSET(gpmc_be1n), (MODE(7) | PULLUDDIS | RXACTIVE)},	        /* GPIO E , Port 1 Pin 28 */
	{-1},
};

static struct module_pin_mux dhcom_gpio2_pin_mux[] = { /* DHCOM GPIOs - F:I (alt SPI2 - see above) */
	{OFFSET(mcasp0_axr0), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO F , Port 3 Pin 16 */
	{OFFSET(mcasp0_fsx), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO G , Port 3 Pin 15 */
	{OFFSET(ecap0_in_pwm0_out), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO H , Port 0 Pin 7 */
	{OFFSET(mcasp0_ahclkr), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO I , Port 3 Pin 17 */
	{-1},
};

static struct module_pin_mux rmii1_pin_mux[] = { /* default eth1 100Mbit on DHCOM */
	{OFFSET(mii1_rxerr), MODE(1) | RXACTIVE | PULLUDEN | PULLUP_EN},/* RMII1_RXERR */
	{OFFSET(mii1_txen), MODE(1) | PULLUDEN},			/* RMII1_TXEN */
	{OFFSET(mii1_crs), MODE(1) | RXACTIVE | PULLUDEN | PULLUP_EN},	/* RMII1_CRS_DV */
	{OFFSET(mii1_txd1), MODE(1) | PULLUDEN},			/* RMII1_TXD1 */
	{OFFSET(mii1_txd0), MODE(1) | PULLUDEN},			/* RMII1_TXD0 */
	{OFFSET(mii1_rxd1), MODE(1) | RXACTIVE | PULLUDEN | PULLUP_EN},	/* RMII1_RXD1 */
	{OFFSET(mii1_rxd0), MODE(1) | RXACTIVE | PULLUDEN | PULLUP_EN},	/* RMII1_RXD0 */
	{OFFSET(rmii1_refclk), MODE(0) | RXACTIVE | PULLUDEN | PULLUP_EN}, /* RMII1_REFCLK */
	{OFFSET(mdio_data), MODE(0) | RXACTIVE | PULLUP_EN}, /* MDIO_DATA */
	{OFFSET(mdio_clk), MODE(0) | PULLUP_EN},	/* MDIO_CLK */
	{-1},
};

/*
 * On DHCOM AM335x you have to choose between 100Mbit eth2 or 1Gbit via DHCOM-X via RGMII
 * If 100Mbit eth2 is required than it is not possible to use NAND
 */
#ifndef CONFIG_GIGABIT_ETH_DHCOMX
static struct module_pin_mux dhcom_gpio3_pin_mux[] = { /* DHCOM GPIOs - J:W  */
	{OFFSET(mcasp0_ahclkx), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO J , Port 3 Pin 21 */
	{OFFSET(gpmc_a9), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO K , Port 1 Pin 25 */
	{OFFSET(gpmc_a8), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO L , Port 1 Pin 24 */
	{OFFSET(gpmc_a7), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO M , Port 1 Pin 23 */
	{OFFSET(gpmc_a6), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO N , Port 1 Pin 22 */
	{OFFSET(gpmc_a3), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO O , Port 1 Pin 19 */
	{OFFSET(gpmc_a2), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO P , Port 1 Pin 18 */
	{OFFSET(gpmc_a1), (MODE(7) | PULLUDDIS | RXACTIVE)},	/* GPIO Q , Port 1 Pin 17 */
	{-1},
};
#ifndef CONFIG_NAND
static struct module_pin_mux rmii2_pin_mux[] = { /* second eth2 100Mbit on DHCOM */
	{OFFSET(gpmc_wpn), MODE(3) | RXACTIVE | PULLUDEN | PULLUP_EN },/* RMII2_RXERR */
	{OFFSET(gpmc_a0), MODE(3)  | PULLUDEN},			/* RMII2_TXEN */
	{OFFSET(gpmc_csn3), MODE(2) | RXACTIVE | PULLUDEN | PULLUP_EN},/* RMII2_CRS_DV */
	{OFFSET(gpmc_a4), MODE(3) | PULLUDEN},			/* RMII2_TXD1 */
	{OFFSET(gpmc_a5), MODE(3) | PULLUDEN},			/* RMII2_TXD0 */
	{OFFSET(gpmc_a10), MODE(3) | RXACTIVE | PULLUDEN | PULLUP_EN}, /* RMII2_RXD1 */
	{OFFSET(gpmc_a11), MODE(3) | RXACTIVE | PULLUDEN | PULLUP_EN}, /* RMII2_RXD0 */
	{OFFSET(mii1_col), MODE(1) | RXACTIVE | PULLUDEN | PULLUP_EN},/* RMII2_REFCLK */
	{OFFSET(mdio_data), MODE(0) | RXACTIVE | PULLUP_EN}, /* MDIO_DATA */
	{OFFSET(mdio_clk), MODE(0) | PULLUP_EN},	/* MDIO_CLK */
	{-1},
};
#endif
#else /* Gbit ethernet via DHCOM-X RGMII */
static struct module_pin_mux rgmii2_pin_mux[] = {
	{OFFSET(gpmc_a0), MODE(2)},			/* RGMII2_TCTL */
	{OFFSET(gpmc_a1), MODE(2) | RXACTIVE},	        /* RGMII2_RCTL */
	{OFFSET(gpmc_a2), MODE(2)},			/* RGMII2_TD3 */
	{OFFSET(gpmc_a3), MODE(2)},			/* RGMII2_TD2 */
	{OFFSET(gpmc_a4), MODE(2)},			/* RGMII2_TD1 */
	{OFFSET(gpmc_a5), MODE(2)},			/* RGMII2_TD0 */
	{OFFSET(gpmc_a6), MODE(2)},			/* RGMII2_TCLK */
	{OFFSET(gpmc_a7), MODE(2) | RXACTIVE},	        /* RGMII2_RCLK */
	{OFFSET(gpmc_a8), MODE(2) | RXACTIVE},	        /* RGMII2_RD3 */
	{OFFSET(gpmc_a9), MODE(2) | RXACTIVE},	        /* RGMII2_RD2 */
	{OFFSET(gpmc_a10), MODE(2) | RXACTIVE},	        /* RGMII2_RD1 */
	{OFFSET(gpmc_a11), MODE(2) | RXACTIVE},	        /* RGMII2_RD0 */
	{OFFSET(mdio_data), MODE(0) | RXACTIVE | PULLUP_EN}, /* MDIO_DATA */
	{OFFSET(mdio_clk), MODE(0) | PULLUP_EN},	/* MDIO_CLK */
	{OFFSET(emu0), MODE(7) | PULLUDDIS },	/* RGMII - INTERRUPT : Port 3 Pin 7 */
	{-1},
};
#endif

#ifdef CONFIG_NAND
static struct module_pin_mux nand_pin_mux[] = { /* NAND on DHCOM (alt to eMMC + eth2) */
	{OFFSET(gpmc_ad0),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD0  */
	{OFFSET(gpmc_ad1),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD1  */
	{OFFSET(gpmc_ad2),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD2  */
	{OFFSET(gpmc_ad3),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD3  */
	{OFFSET(gpmc_ad4),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD4  */
	{OFFSET(gpmc_ad5),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD5  */
	{OFFSET(gpmc_ad6),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD6  */
	{OFFSET(gpmc_ad7),	(MODE(0) | PULLUDDIS | RXACTIVE)}, /* AD7  */
	{OFFSET(gpmc_wait0),	(MODE(0) | PULLUP_EN | RXACTIVE)}, /* nWAIT, #R/B */
	{OFFSET(gpmc_wpn),	(MODE(0) | PULLUP_EN)},		   /* nWP */
	{OFFSET(gpmc_csn0),	(MODE(0) | PULLUP_EN)},		   /* nCS */
	{OFFSET(gpmc_wen),	(MODE(0) | PULLDOWN_EN)},	   /* WEN */
	{OFFSET(gpmc_oen_ren),	(MODE(0) | PULLDOWN_EN)},	   /* OE */
	{OFFSET(gpmc_advn_ale),	(MODE(0) | PULLDOWN_EN)},	   /* ADV_ALE */
	{OFFSET(gpmc_be0n_cle),	(MODE(0) | PULLDOWN_EN)},	   /* BE_CLE */
	{-1},
};
#else /* eMMC is used */
static struct module_pin_mux mmc1_pin_mux[] = { /* eMMC interface on DHCOM */
	{OFFSET(gpmc_ad7), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT7 */
	{OFFSET(gpmc_ad6), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT6 */
	{OFFSET(gpmc_ad5), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT5 */
	{OFFSET(gpmc_ad4), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT4 */
	{OFFSET(gpmc_ad3), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT3 */
	{OFFSET(gpmc_ad2), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT2 */
	{OFFSET(gpmc_ad1), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT1 */
	{OFFSET(gpmc_ad0), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT0 */
	{OFFSET(gpmc_csn1), (MODE(2) | RXACTIVE | PULLUP_EN)},	/* MMC1_CLK */
	{OFFSET(gpmc_csn2), (MODE(2) | RXACTIVE | PULLUP_EN)},	/* MMC1_CMD */
	{-1},
};
#endif

static struct module_pin_mux hw_code_pin_mux[] = { /* Hardware Version Code */
	{OFFSET(gpmc_ad13), (MODE(7) | RXACTIVE | PULLUDDIS)},	        /* LCD_DAT18: HW Code 0 */
	{OFFSET(gpmc_ad12), (MODE(7) | RXACTIVE | PULLUDDIS)},	        /* LCD_DAT19: HW Code 1 */
	{OFFSET(gpmc_ad11), (MODE(7) | RXACTIVE | PULLUDDIS)},	        /* LCD_DAT20: HW Code 2 */
	{-1},
};

static struct module_pin_mux parallel_lcd_pin_mux[] = { /* DHCOM LCD Interface*/
	{OFFSET(lcd_data0), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT0 */
	{OFFSET(lcd_data1), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT1 */
	{OFFSET(lcd_data2), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT2 */
	{OFFSET(lcd_data3), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT3 */
	{OFFSET(lcd_data4), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT4 */
	{OFFSET(lcd_data5), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT5 */
	{OFFSET(lcd_data6), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT6 */
	{OFFSET(lcd_data7), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT7 */
	{OFFSET(lcd_data8), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT8 */
	{OFFSET(lcd_data9), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT9 */
	{OFFSET(lcd_data10), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT10 */
	{OFFSET(lcd_data11), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT11 */
	{OFFSET(lcd_data12), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT12 */
	{OFFSET(lcd_data13), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT13 */
	{OFFSET(lcd_data14), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT14 */
	{OFFSET(lcd_data15), (MODE(0) | PULLUDDIS)},	        /* LCD_DAT15 */
	{OFFSET(gpmc_ad15), (MODE(1) | PULLUDDIS)},	        /* LCD_DAT16 */
	{OFFSET(gpmc_ad14), (MODE(1) | PULLUDDIS)},	        /* LCD_DAT17 */
	{OFFSET(gpmc_ad13), (MODE(1) | PULLUDDIS)},	        /* LCD_DAT18 */
	{OFFSET(gpmc_ad12), (MODE(1) | PULLUDDIS)},	        /* LCD_DAT19 */
	{OFFSET(gpmc_ad11), (MODE(1) | PULLUDDIS)},	        /* LCD_DAT20 */
	{OFFSET(gpmc_ad10), (MODE(1) | PULLUDDIS)},	        /* LCD_DAT21 */
	{OFFSET(gpmc_ad9), (MODE(1) | PULLUDDIS)},	        /* LCD_DAT22 */
	{OFFSET(gpmc_ad8), (MODE(1) | PULLUDDIS)},	        /* LCD_DAT23 */
	{OFFSET(lcd_vsync), (MODE(0) | PULLUDDIS)},	        /* LCD_VSYNC */
	{OFFSET(lcd_hsync), (MODE(0) | PULLUDDIS)},	        /* LCD_HSYNC */
	{OFFSET(lcd_pclk), (MODE(0) | PULLUDDIS)},	        /* LCD_PCLK */
	{OFFSET(lcd_ac_bias_en), (MODE(0) | PULLUDDIS)},	/* LCD_DEN */
	{-1},
};

static struct module_pin_mux usbhost_pin_mux[] = { /* usb1 is used for DHCOM USB Host 1 */
	{OFFSET(usb1_drvvbus), (MODE(0) | PULLUDEN | PULLDOWN_EN)},	/* USB_PWR_EN */
	{OFFSET(mii1_rxdv), (MODE(7) | PULLUDEN | PULLDOWN_EN | RXACTIVE)},	/* USB_HOST_OC: GPIO Port 3 Pin 4 */
	{-1},
};

static struct module_pin_mux pwm_pin_mux[] = { /* pwm - common used for display backlight brightness */
	{OFFSET(mcasp0_aclkx), (MODE(7) | PULLUDDIS)},	/* GPIO_PWM - GPIO 3_14*/
	{-1},
};

void enable_uart0_pin_mux(void)
{
	configure_module_pin_mux(uart0_pin_mux);
}

void enable_uart1_pin_mux(void)
{
	configure_module_pin_mux(uart1_pin_mux);
}

void enable_i2c0_pin_mux(void)
{
	configure_module_pin_mux(i2c0_pin_mux);
}

void enable_pwm_pin_mux(void)
{
	configure_module_pin_mux(pwm_pin_mux);
}

#define HW_CODE_BIT_0   45
#define HW_CODE_BIT_1   44
#define HW_CODE_BIT_2   27

/* little hack to get hw version
 * - hw code pins are part of the lcd interface */
static int hw_code = 0;

void detect_hw_version(void)
{
        configure_module_pin_mux(hw_code_pin_mux);

        gpio_request(HW_CODE_BIT_0, "HW_CODE_BIT_0");
        gpio_request(HW_CODE_BIT_1, "HW_CODE_BIT_1");
        gpio_request(HW_CODE_BIT_2, "HW_CODE_BIT_2");

        gpio_direction_input(HW_CODE_BIT_0);
        gpio_direction_input(HW_CODE_BIT_1);
        gpio_direction_input(HW_CODE_BIT_2);

	hw_code = ((gpio_get_value(HW_CODE_BIT_2) << 2) | (gpio_get_value(HW_CODE_BIT_1) << 1) | gpio_get_value(HW_CODE_BIT_0)) + 1; // HW 100 = 00b; HW 200 = 01b; ...

	configure_module_pin_mux(parallel_lcd_pin_mux);
}

int get_hardware_version(void)
{
        return hw_code;
}

void enable_board_pin_mux()
{
        /* Pinmux DHCOM */
        
        /* UART */
        configure_module_pin_mux(uart0_pin_mux);
        configure_module_pin_mux(uart1_pin_mux);
        
        /* Standard DHCOM GPIOs */
        configure_module_pin_mux(dhcom_gpio1_pin_mux);
        
        /* SPI */
        configure_module_pin_mux(spi0_pin_mux);
#if defined(CONFIG_DHCOM_USE_SPI2)
        configure_module_pin_mux(spi1_pin_mux);
#else /* default is GPIO mode of shared pins */
        configure_module_pin_mux(dhcom_gpio2_pin_mux);
#endif
        
#ifndef CONFIG_GIGABIT_ETH_DHCOMX
        /* Extended DHCOM GPIOs */
        configure_module_pin_mux(dhcom_gpio3_pin_mux);
#endif        
        /* I2C */
        configure_module_pin_mux(i2c0_pin_mux);
        configure_module_pin_mux(i2c2_pin_mux);
        
        /* ext SD / on module µSD */
        configure_module_pin_mux(mmc0_pin_mux);

        /* NAND - eMMC */        
#ifdef CONFIG_NAND
        configure_module_pin_mux(nand_pin_mux);
#else   /* default is eMMC*/
        configure_module_pin_mux(mmc1_pin_mux);
#endif

        /* Ethernet */
        configure_module_pin_mux(rmii1_pin_mux);
#ifndef CONFIG_GIGABIT_ETH_DHCOMX
#ifndef CONFIG_NAND
        /* Have a second 100Mbit Port */
        configure_module_pin_mux(rmii2_pin_mux);
#endif
#else /* Gbit ethernet via DHCOM-X RGMII */
        configure_module_pin_mux(rgmii2_pin_mux);
#endif
        /* Parallel RGB LCD interface */
        configure_module_pin_mux(parallel_lcd_pin_mux);
        /* PWM */
        configure_module_pin_mux(pwm_pin_mux);

        /* USB Host 1 (additional pins) */
        configure_module_pin_mux(usbhost_pin_mux);
}
