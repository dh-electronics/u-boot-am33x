/*
 * board.c
 *
 * Board functions for TI AM335X based boards
 *
 * Copyright (C) 2011, Texas Instruments, Incorporated - http://www.ti.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <spl.h>
#include <asm/arch/cpu.h>
#include <asm/arch/hardware.h>
#include <asm/arch/omap.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mem.h>
#include <asm/io.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <miiphy.h>
#include <cpsw.h>
#include <power/tps65217.h>
#include <power/tps65910.h>
#include <environment.h>
#include <watchdog.h>
#include <environment.h>
#include <lcd.h>
#include "board.h"
#include "../../../drivers/video/am335x-fb.h"
#include "pwm-tiehrpwm.h"

DECLARE_GLOBAL_DATA_PTR;

static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;

unsigned DHCOM_gpios[] = {
	DHCOM_GPIO_A,
	DHCOM_GPIO_B,
	DHCOM_GPIO_C,
	DHCOM_GPIO_D,
	DHCOM_GPIO_E,
	DHCOM_GPIO_F,
	DHCOM_GPIO_G,
	DHCOM_GPIO_H,
	DHCOM_GPIO_I,
	DHCOM_GPIO_J,
	DHCOM_GPIO_K,
	DHCOM_GPIO_L,
	DHCOM_GPIO_M,
	DHCOM_GPIO_N,
	DHCOM_GPIO_O,
	DHCOM_GPIO_P,
	DHCOM_GPIO_Q,
};

#define PWM_BACKLIGHT_GP (32*3+14)

#ifndef CONFIG_SKIP_LOWLEVEL_INIT
static const struct ddr_data ddr3_dhcom_2gb_data = {
	.datardsratio0 = NT5CB128M15FP_RD_DQS,
	.datawdsratio0 = NT5CB128M15FP_WR_DQS,
	.datafwsratio0 = NT5CB128M15FP_PHY_FIFO_WE,
	.datawrsratio0 = NT5CB128M15FP_PHY_WR_DATA,
};

static const struct cmd_control ddr3_dhcom_2gb_cmd_ctrl_data = {
	.cmd0csratio = NT5CB128M15FP_RATIO,
	.cmd0iclkout = NT5CB128M15FP_INVERT_CLKOUT,

	.cmd1csratio = NT5CB128M15FP_RATIO,
	.cmd1iclkout = NT5CB128M15FP_INVERT_CLKOUT,

	.cmd2csratio = NT5CB128M15FP_RATIO,
	.cmd2iclkout = NT5CB128M15FP_INVERT_CLKOUT,
};


/* Nanya 2GB 128Mx16 */
static struct emif_regs ddr3_emif_reg_data_dhcom_2gb = {
	.sdram_config = NT5CB128M15FP_EMIF_SDCFG,
	.ref_ctrl = NT5CB128M15FP_EMIF_SDREF,
	.sdram_tim1 = NT5CB128M15FP_EMIF_TIM1,
	.sdram_tim2 = NT5CB128M15FP_EMIF_TIM2,
	.sdram_tim3 = NT5CB128M15FP_EMIF_TIM3,
	.zq_config = NT5CB128M15FP_ZQ_CFG,
	.emif_ddr_phy_ctlr_1 = NT5CB128M15FP_EMIF_READ_LATENCY,
};


#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
#ifdef CONFIG_SPL_ENV_SUPPORT
        char const *pwm_pol = getenv("pwm_pol");
	const char *normal_pol = "normal";
#endif
	/* break into full u-boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;

#ifdef CONFIG_SPL_ENV_SUPPORT
	env_init();
	env_relocate_spec();
	/* Switch off backlight */
	if(pwm_pol) {
		if (!strcmp(pwm_pol, normal_pol)) {
			gpio_direction_output(PWM_BACKLIGHT_GP, 0);
		} else {
			gpio_direction_output(PWM_BACKLIGHT_GP, 1);
		}
	}
	if (getenv_yesno("boot_os") != 1)
		return 1;
#endif

	return 0;
}
#endif

#define OSC	(V_OSCK/1000000)
const struct dpll_params dpll_ddr_dhcom = {
		400, OSC-1, 1, -1, -1, -1, -1};

void am33xx_spl_board_init(void)
{
	int mpu_vdd;
	struct cm_perpll *const cmper = (struct cm_perpll *)CM_PER;
	
	/*
	 * enable additional clocks 
	 */
	u32 *const clk_domains[] = { 0 };

	u32 *const clk_modules_dhcomspecific[] = {
		&cmper->epwmss0clkctrl,
		&cmper->epwmss1clkctrl,
		&cmper->epwmss2clkctrl,
		&cmper->lcdclkctrl,
		&cmper->lcdcclkstctrl,
		0
	};
	do_enable_clocks(clk_domains, clk_modules_dhcomspecific, 1);
	/* setup LCD-Pixel Clock */
	writel(0x2, CM_DPLL + 0x34);
	
	/* power-OFF LCD-Display - in inverted pwm case this disables the backlight */
        enable_pwm_pin_mux();
	gpio_direction_output(PWM_BACKLIGHT_GP, 1);

	/* Get the frequency */
	dpll_mpu_opp100.m = am335x_get_efuse_mpu_max_freq(cdev);

	/* DHCOM AM335x PMIC Code */
	int usb_cur_lim;

        enable_i2c0_pin_mux();
        i2c_init(CONFIG_SYS_OMAP24_I2C_SPEED, CONFIG_SYS_OMAP24_I2C_SLAVE);

	if (i2c_probe(TPS65217_CHIP_PM))
		return;

	/*
	 * Increase USB current limit to 1300mA or 1800mA and set
	 * the MPU voltage controller as needed.
	 */
	if (dpll_mpu_opp100.m == MPUPLL_M_1000) {
		usb_cur_lim = TPS65217_USB_INPUT_CUR_LIMIT_1800MA;
		mpu_vdd = TPS65217_DCDC_VOLT_SEL_1325MV;
	} else {
		usb_cur_lim = TPS65217_USB_INPUT_CUR_LIMIT_1300MA;
		mpu_vdd = TPS65217_DCDC_VOLT_SEL_1275MV;
	}

	if (tps65217_reg_write(TPS65217_PROT_LEVEL_NONE,
			       TPS65217_POWER_PATH,
			       usb_cur_lim,
			       TPS65217_USB_INPUT_CUR_LIMIT_MASK))
		puts("tps65217_reg_write failure\n");

	/* Set DCDC3 (CORE) voltage to 1.125V */
	if (tps65217_voltage_update(TPS65217_DEFDCDC3,
				    TPS65217_DCDC_VOLT_SEL_1125MV)) {
		puts("tps65217_voltage_update failure\n");
		return;
	}

	/* Set CORE Frequencies to OPP100 */
	do_setup_dpll(&dpll_core_regs, &dpll_core_opp100);

	/* Set DCDC2 (MPU) voltage */
	if (tps65217_voltage_update(TPS65217_DEFDCDC2, mpu_vdd)) {
		puts("tps65217_voltage_update failure\n");
		return;
	}

	/*
	 * Set LDO3 to 1.8V and LDO4 to 3.3V for DHCOM AM335x.
	 */
        if (tps65217_reg_write(TPS65217_PROT_LEVEL_2,
			       TPS65217_DEFLS1,
			       TPS65217_LDO_VOLTAGE_OUT_1_8,
			       TPS65217_LDO_MASK))
		puts("tps65217_reg_write failure\n");

	if (tps65217_reg_write(TPS65217_PROT_LEVEL_2,
			       TPS65217_DEFLS2,
			       TPS65217_LDO_VOLTAGE_OUT_3_3,
			       TPS65217_LDO_MASK))
		puts("tps65217_reg_write failure\n");

	/* Set MPU Frequency to what we detected now that voltages are set */
	do_setup_dpll(&dpll_mpu_regs, &dpll_mpu_opp100);
}

const struct dpll_params *get_dpll_ddr_params(void)
{
	return &dpll_ddr_dhcom;
}


void set_uart_mux_conf(void)
{
#if CONFIG_CONS_INDEX == 1
	enable_uart0_pin_mux();
#elif CONFIG_CONS_INDEX == 2
	enable_uart1_pin_mux();
#endif
}

void set_mux_conf_regs(void)
{
	enable_board_pin_mux();
}

const struct ctrl_ioregs ioregs_dhcom_2gb = {
	.cm0ioctl		= NT5CB128M15FP_IOCTRL_VALUE,
	.cm1ioctl		= NT5CB128M15FP_IOCTRL_VALUE,
	.cm2ioctl		= NT5CB128M15FP_IOCTRL_VALUE,
	.dt0ioctl		= NT5CB128M15FP_IOCTRL_VALUE,
	.dt1ioctl		= NT5CB128M15FP_IOCTRL_VALUE,
};

void sdram_init(void)
{
 	config_ddr(400, &ioregs_dhcom_2gb,
			   &ddr3_dhcom_2gb_data,
			   &ddr3_dhcom_2gb_cmd_ctrl_data,
			   &ddr3_emif_reg_data_dhcom_2gb, 0);
}
#endif

/*
 * Basic board specific setup.  Pinmux has been handled already.
 */
int board_init(void)
{
#if defined(CONFIG_HW_WATCHDOG)
	hw_watchdog_init();
#endif

	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
#if defined(CONFIG_NOR) || defined(CONFIG_NAND)
	gpmc_init();
#endif
	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
	return 0;
}
#endif

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
static void cpsw_control(int enabled)
{
	/* VTP can be added here */

	return;
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 0,
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= 1,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 1,
	.slave_data		= cpsw_slaves,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.bd_ram_ofs		= 0x2000,
	.mac_control		= (1 << 5),
	.control		= cpsw_control,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};
#endif

/*
 * This function will:
 * Read the eFuse for MAC addresses, and set ethaddr/eth1addr/usbnet_devaddr
 * in the environment
 * Perform fixups to the PHY present on certain boards.  We only need this
 * function in:
 * - SPL with either CPSW or USB ethernet support
 * - Full U-Boot, with either CPSW or USB ethernet
 * Build in only these cases to avoid warnings about unused variables
 * when we build an SPL that has neither option but full U-Boot will.
 */
#if ((defined(CONFIG_SPL_ETH_SUPPORT) || defined(CONFIG_SPL_USBETH_SUPPORT)) \
		&& defined(CONFIG_SPL_BUILD)) || \
	((defined(CONFIG_DRIVER_TI_CPSW) || \
	  defined(CONFIG_USB_ETHER) && defined(CONFIG_MUSB_GADGET)) && \
	 !defined(CONFIG_SPL_BUILD))
int board_eth_init(bd_t *bis)
{
        /*
         * dhcom am335x:
         *      VIO voltage is connected to 3V3 (not switchable)
         *      Reset lines of the ethernet phy are connected to system reset line
         */
	int rv, n = 0;
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;

	/* try reading mac address from efuse */
	mac_lo = readl(&cdev->macid0l);
	mac_hi = readl(&cdev->macid0h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
	if (!getenv("ethaddr")) {
		printf("<ethaddr> not set. Validating first E-fuse MAC\n");

		if (is_valid_ether_addr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
	}

#ifdef CONFIG_DRIVER_TI_CPSW

	mac_lo = readl(&cdev->macid1l);
	mac_hi = readl(&cdev->macid1h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

	if (!getenv("eth1addr")) {
		if (is_valid_ether_addr(mac_addr))
			eth_setenv_enetaddr("eth1addr", mac_addr);
	}

        /* Set RMII Mode for RMII1 & RMII2, set REFCLK to input mode */
	writel(RMII_MODE_ENABLE | RMII_CHIPCKL_ENABLE, &cdev->miisel);
	cpsw_slaves[0].phy_if = cpsw_slaves[1].phy_if =
			PHY_INTERFACE_MODE_RMII;


	rv = cpsw_register(&cpsw_data);
	if (rv < 0)
		printf("Error %d registering CPSW switch\n", rv);
	else
		n += rv;
#endif
#endif

#if defined(CONFIG_USB_ETHER) && \
	(!defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_USBETH_SUPPORT))
	if (is_valid_ether_addr(mac_addr))
		eth_setenv_enetaddr("usbnet_devaddr", mac_addr);

	rv = usb_eth_initialize(bis);
	if (rv < 0)
		printf("Error %d registering USB_ETHER\n", rv);
	else
		n += rv;
#endif
	return n;
}
#endif

#if defined(CONFIG_SPLASH_SCREEN) && !defined(CONFIG_SPL_BUILD)
static int board_get_splashimage(void)
{
        char *s;
        char *command;

        s = getenv("splashimage");
	if (s) { /* only load splash image if we have env var 'splashimage' */

                /* disable console output */
                gd->flags |= GD_FLG_DISABLE_CONSOLE;

                /* load splash image bmp file from a file system */
                command = getenv ("load_splash");
	        if (command == NULL) {
                        /* enable console output */
                        gd->flags &= (~GD_FLG_DISABLE_CONSOLE);
                        printf ("splashimage: \"load_splash\" not defined\n");
                        return -ENOENT;
                }
                else if (run_command (command, 0) != 0) {
                        /* enable console output */
                        gd->flags &= (~GD_FLG_DISABLE_CONSOLE);
                        printf ("Warning: Can't load splash bitmap\n");
                        return -EIO;
                }

	        /* enable console output */
	        gd->flags &= (~GD_FLG_DISABLE_CONSOLE);
	}
	return 0;
}
#endif

void set_dhcom_defaultsettings(volatile settingsinfo_t* dh_settings)
{
	/* initialize DH Global Data */
	dh_settings->cLength = DEFAULT_SETTINGS_BLOCK_LENGTH;
	dh_settings->cDisplayID = DEFAULT_SETTINGS_DISPLAY_ID;
	dh_settings->wValidationID = 0;
	dh_settings->wYResolution = DEFAULT_SETTINGS_Y_RESOLUTION;
	dh_settings->wXResolution = DEFAULT_SETTINGS_X_RESOLUTION;
	dh_settings->wLCDConfigFlags = DEFAULT_SETTINGS_LCD_CONFIG_FLAGS;
	dh_settings->wPixelClock = DEFAULT_SETTINGS_PIXEL_CLOCK;
	dh_settings->wVPulseWidth = DEFAULT_SETTINGS_V_PULSE_WIDTH;
	dh_settings->wHPulseWidth = DEFAULT_SETTINGS_H_PULSE_WIDTH;
	dh_settings->wHBackPorch = DEFAULT_SETTINGS_H_BACK_PORCH;
	dh_settings->wHFrontPorch = DEFAULT_SETTINGS_H_FRONT_PORCH;
	dh_settings->wVBackPorch = DEFAULT_SETTINGS_V_BACK_PORCH;
	dh_settings->wVFrontPorch = DEFAULT_SETTINGS_V_FRONT_PORCH;
	dh_settings->cACBiasTrans = DEFAULT_SETTINGS_AC_BIAS_TRANS;
	dh_settings->cACBiasFreq = DEFAULT_SETTINGS_AC_BIAS_FREQ;
	dh_settings->cDatalines = DEFAULT_SETTINGS_DATALINES;
	dh_settings->wGPIODir = DEFAULT_SETTINGS_GPIO_DIRECTION;
	dh_settings->wGPIOState = DEFAULT_SETTINGS_GPIO_STATE;
	dh_settings->wHWConfigFlags = DEFAULT_SETTINGS_HW_CONFIG_FLAGS;
}

void set_dhcom_da_eeprom_settings(volatile settingsinfo_t* dh_settings)
{
	int old_bus, ret_value;
        uchar ucBuffer[DHCOM_DISPLAY_SETTINGS_SIZE];

        /* prepare i2c bus */
        old_bus = I2C_GET_BUS();
        I2C_SET_BUS(DISPLAY_ADAPTER_EEPROM_I2C_BUS);
	i2c_init(CONFIG_SYS_OMAP24_I2C_SPEED, CONFIG_SYS_OMAP24_I2C_SLAVE);
	
	/* read display settings */
	ret_value = i2c_read(DISPLAY_ADAPTER_EEPROM_ADDR, 0, 1, &ucBuffer[0], DHCOM_DISPLAY_SETTINGS_SIZE);
	if((ret_value == 0) && (ucBuffer[2] == 'D') && (ucBuffer[3] == 'H')) {
		dh_settings->cDisplayID = ucBuffer[1];
                dh_settings->wValidationID = 0x4144; /* set 'DA' for display adapter*/
		dh_settings->wYResolution = (ucBuffer[5] << 8) | ucBuffer[4];
		dh_settings->wXResolution = (ucBuffer[7] << 8) | ucBuffer[6];

		dh_settings->wLCDConfigFlags = (ucBuffer[9] << 8) | ucBuffer[8];
		dh_settings->wPixelClock = (ucBuffer[11] << 8) | ucBuffer[10];

		dh_settings->wVPulseWidth = (ucBuffer[13] << 8) | ucBuffer[12];
		dh_settings->wHPulseWidth = (ucBuffer[15] << 8) | ucBuffer[14];

		dh_settings->wHBackPorch = (ucBuffer[17] << 8) | ucBuffer[16];
		dh_settings->wHFrontPorch = (ucBuffer[19] << 8) | ucBuffer[18];

		dh_settings->wVBackPorch = (ucBuffer[21] << 8) | ucBuffer[20];
		dh_settings->wVFrontPorch = (ucBuffer[23] << 8) | ucBuffer[22];

		dh_settings->cACBiasTrans = ucBuffer[24];
		dh_settings->cACBiasFreq = ucBuffer[25];
		dh_settings->cDatalines = (ucBuffer[27] << 8) | ucBuffer[26];
	}

	/* Set i2c bus back */	
	I2C_SET_BUS(old_bus);
}

void load_dh_settings_file(void)
{
	ulong addr;
	char *command;

        set_dhcom_defaultsettings(&gd->dh_board_settings);
        
	addr = simple_strtoul(getenv ("loadaddr"), NULL, 16);
	printf("loading dhcom settings...\n");

	// Disable console output
	gd->flags |= GD_FLG_DISABLE_CONSOLE;

	/* Load DH settings file from Filesystem */
	if ((command = getenv ("load_settings_bin")) == NULL) {
		/* Error: no environment variable 'load_settings_bin' */
		gd->flags &= (~GD_FLG_DISABLE_CONSOLE);	
		printf ("Warning: \"load_settings_bin\" not defined\n");
	}
	else if (run_command (command, 0) != 0) {
		/* Error while executing 'load_settings_bin' */
		gd->flags &= (~GD_FLG_DISABLE_CONSOLE);	
		printf ("Warning: Can't load dhcom settings file\n");		
	}
	else {
	        /* copy settingsblock from dram into the dh_board_settings structure */
	        gd->dh_board_settings.wValidationID = ((readl(addr) & 0xFFFF0000) >> 16);

	        // settings.bin file Valid Mask should be "DH" = 0x4844
	        if(gd->dh_board_settings.wValidationID == 0x4844) {
		        gd->dh_board_settings.cLength = (readl(addr) & 0xFF);
		        gd->dh_board_settings.cDisplayID = ((readl(addr) & 0xFF00) >> 8);

		        gd->dh_board_settings.wYResolution = (readl(addr+4) & 0xFFFF);
		        gd->dh_board_settings.wXResolution = ((readl(addr+4) & 0xFFFF0000) >> 16);

		        gd->dh_board_settings.wLCDConfigFlags = (readl(addr+8) & 0xFFFF);
		        gd->dh_board_settings.wPixelClock = ((readl(addr+8) & 0xFFFF0000) >> 16);

		        gd->dh_board_settings.wVPulseWidth = (readl(addr+12) & 0xFFFF);
		        gd->dh_board_settings.wHPulseWidth = ((readl(addr+12) & 0xFFFF0000) >> 16);

		        gd->dh_board_settings.wHBackPorch = (readl(addr+16) & 0xFFFF);
		        gd->dh_board_settings.wHFrontPorch = ((readl(addr+16) & 0xFFFF0000) >> 16);

		        gd->dh_board_settings.wVBackPorch = (readl(addr+20) & 0xFFFF);
		        gd->dh_board_settings.wVFrontPorch = ((readl(addr+20) & 0xFFFF0000) >> 16);

		        gd->dh_board_settings.cACBiasTrans = (readl(addr+24) & 0xFF);
		        gd->dh_board_settings.cACBiasFreq = ((readl(addr+24) & 0xFF00) >> 8);
		        gd->dh_board_settings.cDatalines = ((readl(addr+24) & 0xFFFF0000) >> 16);

		        gd->dh_board_settings.wGPIODir = (readl(addr+32));
		        gd->dh_board_settings.wGPIOState = (readl(addr+36));

		        gd->dh_board_settings.wHWConfigFlags = (readl(addr+40) & 0xFFFF);
	        }	
	        else {
		        gd->dh_board_settings.wValidationID = 0;
	        }
	}

	/* Enable console output */
	gd->flags &= (~GD_FLG_DISABLE_CONSOLE);	

	/* Check and Read Display data from EEPROM if enabled */
	if((gd->dh_board_settings.wHWConfigFlags & SETTINGS_HW_EN_DISP_ADPT_EE_CHK) != 0) {
		set_dhcom_da_eeprom_settings(&gd->dh_board_settings);
	}
			
}

void set_dhcom_gpios(void)
{
	int i;
	unsigned int mask = 0x1;
	char labeltext[7] = {"GPIO_A\0"};

	for(i = 0; i < 17; i++) {
	        gpio_request(DHCOM_gpios[i], labeltext);
	        labeltext[5]++;
		if(gd->dh_board_settings.wGPIODir & mask) {
		        /* Set to input */
			gpio_direction_input(DHCOM_gpios[i]);		
		} else {
		        /* Set to output */
			if(gd->dh_board_settings.wGPIOState & mask)
				gpio_direction_output(DHCOM_gpios[i] , 1);
			else
				gpio_direction_output(DHCOM_gpios[i] , 0);
		}
		mask = mask << 1;
	}
}

void set_dhcom_backlight_gpio(void)
{
	int backlight_gpio;
	int backlight_en_pol;
	int backlight_pwm_pol;
	char labeltext[19] = {"GPIO_PWM_BACKLIGHT\0"};

	// Mask Backlight enable GPIO
	backlight_gpio = ((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_BL_EN_GPIO_FLAG) >> 7);

	// Check if backlight enable ist specified
	if(backlight_gpio != 0) {
		// Covert to struct gpio index
		backlight_gpio = backlight_gpio - 1;

		// Mask Backlight pol flag: 0 = active high; 1 = active low
		backlight_en_pol = (gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_IBL_FLAG);

		// Mask Backlight PWM pol flag: 0 = active high; 1 = active low
		backlight_pwm_pol = (gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_PWM_POL_FLAG);

		// Enable backlight - gpio alread requested on DHCOM GPIO setup
		if(backlight_en_pol == 0) {
			gpio_direction_output(DHCOM_gpios[backlight_gpio] , 1);
		} else {
			gpio_direction_output(DHCOM_gpios[backlight_gpio] , 0);
		}
#ifdef PWM_BACKLIGHT_GP
		// Enable backlight PWM pin
		gpio_request(PWM_BACKLIGHT_GP, labeltext);
		if(backlight_pwm_pol == 0) {
			gpio_direction_output(PWM_BACKLIGHT_GP, 1);
			setenv("pwm_pol", "normal");
		} else {
			gpio_direction_output(PWM_BACKLIGHT_GP, 0);
			setenv("pwm_pol", "inverted");
		}
#endif	
	}
}

/*
 * init hook which is called immediatly before lcd setup
 * and after nand/mmc setup
 */
int dhcom_init(void)
{
        load_dh_settings_file();
	set_dhcom_gpios();
        return 0;
}

#if defined(CONFIG_LCD) && defined(CONFIG_AM335X_LCD) && \
	!defined(CONFIG_SPL_BUILD)
int load_lcdtiming(struct am335x_lcdpanel *panel, volatile settingsinfo_t *dh_settings)
{
	struct am335x_lcdpanel pnltmp;

	pnltmp.hactive = dh_settings->wXResolution;
	pnltmp.vactive = dh_settings->wYResolution;
	pnltmp.bpp = 32; /* Always setup 32 bpp framebuffer */
	pnltmp.hfp = dh_settings->wHFrontPorch;
	pnltmp.hbp = dh_settings->wHBackPorch;
	pnltmp.hsw = dh_settings->wHPulseWidth;
	pnltmp.vfp = dh_settings->wVFrontPorch;
	pnltmp.vbp = dh_settings->wVBackPorch;
	pnltmp.vsw = dh_settings->wVPulseWidth;
	pnltmp.pxl_clk_div = 192000 / dh_settings->wPixelClock;

	/* setup pol flags */
        pnltmp.pol = 0;

        if(dh_settings->wLCDConfigFlags & SETTINGS_LCD_IVS_FLAG) {
                pnltmp.pol |= VSYNC_INVERT;
        }

        if(dh_settings->wLCDConfigFlags & SETTINGS_LCD_IHS_FLAG) {
                pnltmp.pol |= HSYNC_INVERT;
        }

        if(!(dh_settings->wLCDConfigFlags & SETTINGS_LCD_IPC_FLAG)) {
                pnltmp.pol |= PXCLK_INVERT;
        }

        if(dh_settings->wLCDConfigFlags & SETTINGS_LCD_IOE_FLAG) {
                pnltmp.pol |= DE_INVERT;
        }

        if(dh_settings->wLCDConfigFlags & SETTINGS_LCD_IDATA_FLAG) {
                printf("LCD: inverted data is not supported!\n");
        }

        if(!(dh_settings->wLCDConfigFlags & SETTINGS_LCD_ACT_PAS_FLAG)) {
                printf("LCD: passive matrix displays are not supported!\n");
        }

	pnltmp.pup_delay = 0; 
	pnltmp.pon_delay = 0;

	/* TODO: setup datalines dh_settings->cDatalines */

	memcpy((void *)panel,
	       (void *)&pnltmp,
	       sizeof(struct am335x_lcdpanel));

	return 0;
}

void lcdpower(int on)
{
        /* dummy */
}

vidinfo_t	panel_info = {
		.vl_col = 2048,	/*
				 * give full resolution for allocating enough
				 * memory
				 */
		.vl_row = 2048,
		.vl_bpix = 5,
		.priv = 0
};

void lcd_ctrl_init(void *lcdbase)
{
	struct am335x_lcdpanel lcd_panel;

#ifdef CONFIG_SPLASH_SCREEN
        board_get_splashimage();
#endif
	memset(&lcd_panel, 0, sizeof(struct am335x_lcdpanel));
	if (load_lcdtiming(&lcd_panel, &gd->dh_board_settings) != 0)
		return;

	lcd_panel.panel_power_ctrl = &lcdpower;

	if (0 != am335xfb_init(&lcd_panel)) {
		printf("ERROR: failed to initialize video!");
	} 
	/*
	 * modifiy panel info to 'real' resolution, to operate correct with
	 * lcd-framework.
	 */
	panel_info.vl_col = lcd_panel.hactive;
	panel_info.vl_row = lcd_panel.vactive;

	lcd_set_flush_dcache(1);
}

#ifndef PWM_BACKLIGHT_GP
/*
 * TODO: this needs some work
 */
static int ehrpwm_pwm_enable(unsigned int bright)
{
	unsigned short *epwm0_base = (unsigned short*)0x48300200;

        /* scale time base clk relative to sys clock */
        *(epwm0_base + TBCTL/2) = TBCTL_CTRMODE_DOWN | 
                                TBCTL_SYNCOSEL_ZERO | 
                                (0x05 << 7) | (0x02 << 10) |
                                TBCTL_PRDLD_IMDT;

        /* configure counter value */
        *(epwm0_base + TBCNT/2) = 100;
        *(epwm0_base + TBPRD/2) = 100;
        
        /* compare control */
        *(epwm0_base + CMPCTL/2) = 0;
        *(epwm0_base + CMPA/2) = bright;
        *(epwm0_base + CMPB/2) = 0;
        
	/* Action Qualifier on PWM output A */
	*(epwm0_base + AQCTLA/2) = AQCTL_CBD_FRCLOW | AQCTL_CAD_FRCHIGH;
        
        /* scale time base clk relative to sys clock */
        *(epwm0_base + TBCTL/2) = TBCTL_CTRMODE_DOWN | 
                                TBCTL_SYNCOSEL_ZERO | 
                                (0x05 << 7) | (0x02 << 10) |
                                TBCTL_FREE_RUN;
	return 0;
}
#endif

void lcd_enable(void)
{
#ifndef PWM_BACKLIGHT_GP
	unsigned int bright = getenv_ulong("bright_duty", 10, 50);
		
	bright = bright != ~0UL ? bright : 50;
	
	ehrpwm_pwm_enable(bright);
#endif
	set_dhcom_backlight_gpio();
}
#elif CONFIG_SPL_BUILD
#else
#error "LCD-support with a suitable FB-Driver is mandatory !"
#endif /* CONFIG_LCD */
