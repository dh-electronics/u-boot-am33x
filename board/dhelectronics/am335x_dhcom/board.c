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

DECLARE_GLOBAL_DATA_PTR;

static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;

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
	/* break into full u-boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;

#ifdef CONFIG_SPL_ENV_SUPPORT
	env_init();
	env_relocate_spec();
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

	u32 *const clk_modules_kwbspecific[] = {
		&cmper->epwmss0clkctrl,
		&cmper->epwmss1clkctrl,
		&cmper->epwmss2clkctrl,
		&cmper->lcdclkctrl,
		&cmper->lcdcclkstctrl,
		0
	};
	do_enable_clocks(clk_domains, clk_modules_kwbspecific, 1);
	/* setup LCD-Pixel Clock */
	writel(0x2, CM_DPLL + 0x34);
	
	/* power-OFF LCD-Display */
	// gpio_direction_output(LCD_PWR, 0);

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
	__maybe_unused struct am335x_baseboard_id header;

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

#ifdef CONFIG_USE_FDT
  #define FDTPROP(a, b, c) fdt_getprop_u32_default((void *)a, b, c, ~0UL)
  #define PATHTIM "/panel/display-timings/default"
  #define PATHINF "/panel/panel-info"
#endif
/* --------------------------------------------------------------------------*/
#if defined(CONFIG_LCD) && defined(CONFIG_AM335X_LCD) && \
	!defined(CONFIG_SPL_BUILD)
int load_lcdtiming(struct am335x_lcdpanel *panel)
{
	struct am335x_lcdpanel pnltmp;
#ifdef CONFIG_USE_FDT_DO_NOT
	u32 dtbaddr = getenv_ulong("dtbaddr", 16, ~0UL);
	u32 dtbprop;

	if (dtbaddr == ~0UL) {
		puts("load_lcdtiming: failed to get 'dtbaddr' from env!\n");
		return -1;
	}
	memcpy(&pnltmp, (void *)panel, sizeof(struct am335x_lcdpanel));

	pnltmp.hactive = FDTPROP(dtbaddr, PATHTIM, "hactive");
	pnltmp.vactive = FDTPROP(dtbaddr, PATHTIM, "vactive");
	pnltmp.bpp = FDTPROP(dtbaddr, PATHINF, "bpp");
	pnltmp.hfp = FDTPROP(dtbaddr, PATHTIM, "hfront-porch");
	pnltmp.hbp = FDTPROP(dtbaddr, PATHTIM, "hback-porch");
	pnltmp.hsw = FDTPROP(dtbaddr, PATHTIM, "hsync-len");
	pnltmp.vfp = FDTPROP(dtbaddr, PATHTIM, "vfront-porch");
	pnltmp.vbp = FDTPROP(dtbaddr, PATHTIM, "vback-porch");
	pnltmp.vsw = FDTPROP(dtbaddr, PATHTIM, "vsync-len");
	pnltmp.pup_delay = FDTPROP(dtbaddr, PATHTIM, "pupdelay");
	pnltmp.pon_delay = FDTPROP(dtbaddr, PATHTIM, "pondelay");

	/* calc. proper clk-divisor */
	dtbprop = FDTPROP(dtbaddr, PATHTIM, "clock-frequency");
	if (dtbprop != ~0UL)
		pnltmp.pxl_clk_div = 192000000 / dtbprop;
	else
		pnltmp.pxl_clk_div = ~0UL;

	/* check polarity of control-signals */
	dtbprop = FDTPROP(dtbaddr, PATHTIM, "hsync-active");
	if (dtbprop == 0)
		pnltmp.pol |= HSYNC_INVERT;
	dtbprop = FDTPROP(dtbaddr, PATHTIM, "vsync-active");
	if (dtbprop == 0)
		pnltmp.pol |= VSYNC_INVERT;
	dtbprop = FDTPROP(dtbaddr, PATHINF, "sync-ctrl");
	if (dtbprop == 1)
		pnltmp.pol |= HSVS_CONTROL;
	dtbprop = FDTPROP(dtbaddr, PATHINF, "sync-edge");
	if (dtbprop == 1)
		pnltmp.pol |= HSVS_RISEFALL;
	dtbprop = FDTPROP(dtbaddr, PATHTIM, "pixelclk-active");
	if (dtbprop == 0)
		pnltmp.pol |= PXCLK_INVERT;
	dtbprop = FDTPROP(dtbaddr, PATHTIM, "de-active");
	if (dtbprop == 0)
		pnltmp.pol |= DE_INVERT;
#else
	pnltmp.hactive = 800;
	pnltmp.vactive = 480;
	pnltmp.bpp = 32;
	pnltmp.hfp = 42;
	pnltmp.hbp = 86;
	pnltmp.hsw = 128;
	pnltmp.vfp = 10;
	pnltmp.vbp = 33;
	pnltmp.vsw = 2;
	pnltmp.pxl_clk_div = 192000000 / 33260000;
	pnltmp.pol = HSYNC_INVERT | VSYNC_INVERT | PXCLK_INVERT /* | DE_INVERT */;
	pnltmp.pup_delay = 0; 
	pnltmp.pon_delay = 0;
#endif
	if (
	   ~0UL == (pnltmp.hactive) ||
	   ~0UL == (pnltmp.vactive) ||
	   ~0UL == (pnltmp.bpp) ||
	   ~0UL == (pnltmp.hfp) ||
	   ~0UL == (pnltmp.hbp) ||
	   ~0UL == (pnltmp.hsw) ||
	   ~0UL == (pnltmp.vfp) ||
	   ~0UL == (pnltmp.vbp) ||
	   ~0UL == (pnltmp.vsw) ||
	   ~0UL == (pnltmp.pxl_clk_div) ||
	   ~0UL == (pnltmp.pol) ||
	   ~0UL == (pnltmp.pup_delay) ||
	   ~0UL == (pnltmp.pon_delay)
	   ) {
		puts("lcd-settings in env/dtb incomplete!\n");
		printf("display-timings:\n"
			"================\n"
			"hactive: %d\n"
			"vactive: %d\n"
			"bpp    : %d\n"
			"hfp    : %d\n"
			"hbp    : %d\n"
			"hsw    : %d\n"
			"vfp    : %d\n"
			"vbp    : %d\n"
			"vsw    : %d\n"
			"pxlclk : %d\n"
			"pol    : 0x%08x\n"
			"pondly : %d\n",
			pnltmp.hactive, pnltmp.vactive, pnltmp.bpp,
			pnltmp.hfp, pnltmp.hbp, pnltmp.hsw,
			pnltmp.vfp, pnltmp.vbp, pnltmp.vsw,
			pnltmp.pxl_clk_div, pnltmp.pol, pnltmp.pon_delay);

		return -1;
	}
	debug("lcd-settings in env complete, taking over.\n");
	memcpy((void *)panel,
	       (void *)&pnltmp,
	       sizeof(struct am335x_lcdpanel));

	return 0;
}

#ifdef CONFIG_USE_FDT_DO_NOT
static int load_devicetree(void)
{
	char *dtbname = getenv("dtb");
	char *dtbdev = getenv("dtbdev");
	char *dtppart = getenv("dtbpart");
	u32 dtbaddr = getenv_ulong("dtbaddr", 16, ~0UL);
	loff_t dtbsize;

	if (!dtbdev || !dtbdev) {
		puts("load_devicetree: <dtbdev>/<dtbpart> missing.\n");
		return -1;
	}

	if (fs_set_blk_dev(dtbdev, dtppart, FS_TYPE_EXT)) {
		puts("load_devicetree: set_blk_dev failed.\n");
		return -1;
	}
	if (dtbname && dtbaddr != ~0UL) {
		if (fs_read(dtbname, dtbaddr, 0, 0, &dtbsize) == 0) {
			gd->fdt_blob = (void *)dtbaddr;
			gd->fdt_size = dtbsize;
			debug("loaded %d bytes of dtb onto 0x%08x\n",
			      (u32)dtbsize, dtbaddr);
			return dtbsize;
		}
		puts("load_devicetree: load dtb failed,file does not exist!\n");
	}

	puts("load_devicetree: <dtb>/<dtbaddr> missing!\n");
	return -1;
}
#endif

void lcdpower(int on)
{
	u32 pin, swval, i;
#ifdef CONFIG_USE_FDT_DO_NOT
	u32 dtbaddr = getenv_ulong("dtbaddr", 16, ~0UL);

	if (dtbaddr == ~0UL) {
		puts("lcdpower: failed to get 'dtbaddr' from env!\n");
		return;
	}
	pin = FDTPROP(dtbaddr, PATHINF, "pwrpin");
#else
	pin = getenv_ulong("ds1_pwr", 16, ~0UL);
#endif
	if (pin == ~0UL) {
		/* puts("no pwrpin in dtb/env, cannot powerup display!\n"); */
		pin = 111;
		return;
	}

	for (i = 0; i < 3; i++) {
		if (pin != 0) {
			swval = pin & 0x80 ? 0 : 1;
			if (on)
				gpio_direction_output(pin & 0x7F, swval);
			else
				gpio_direction_output(pin & 0x7F, !swval);

			debug("switched pin %d to %d\n", pin & 0x7F, swval);
		}
		pin >>= 8;
	}
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
#ifdef CONFIG_USE_FDT_DO_NOT
	/* TODO: is there a better place to load the dtb ? */
	load_devicetree();
#endif
        
	memset(&lcd_panel, 0, sizeof(struct am335x_lcdpanel));
	if (load_lcdtiming(&lcd_panel) != 0)
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

void lcd_enable(void)
{
#ifdef DO_NET_EXECUTE
#ifdef CONFIG_USE_FDT
	u32 dtbaddr = getenv_ulong("dtbaddr", 16, ~0UL);

	if (dtbaddr == ~0UL) {
		puts("lcdpower: failed to get 'dtbaddr' from env!\n");
		return;
	}
	unsigned int driver = FDTPROP(dtbaddr, PATHINF, "brightdrv");
	unsigned int bright = FDTPROP(dtbaddr, PATHINF, "brightdef");
	unsigned int pwmfrq = FDTPROP(dtbaddr, PATHINF, "brightfdim");
#else
	unsigned int driver = getenv_ulong("ds1_bright_drv", 16, 0UL);
	unsigned int bright = getenv_ulong("ds1_bright_def", 10, 50);
	unsigned int pwmfrq = getenv_ulong("ds1_pwmfreq", 10, ~0UL);
#endif
	unsigned int tmp;
	struct gptimer *const timerhw = (struct gptimer *)DM_TIMER6_BASE;

	bright = bright != ~0UL ? bright : 50;

	switch (driver) {
	case 0:	/* PMIC LED-Driver */
		/* brightness level */
		tps65217_reg_write(TPS65217_PROT_LEVEL_NONE,
				   TPS65217_WLEDCTRL2, bright, 0xFF);
		/* turn on light */
		tps65217_reg_write(TPS65217_PROT_LEVEL_NONE,
				   TPS65217_WLEDCTRL1, 0x0A, 0xFF);
		break;
	case 1: /* PWM using timer6 */
		if (pwmfrq != ~0UL) {
			timerhw->tiocp_cfg = TCFG_RESET;
			udelay(10);
			while (timerhw->tiocp_cfg & TCFG_RESET)
				;
			tmp = ~0UL-(V_OSCK/pwmfrq);	/* bottom value */
			timerhw->tldr = tmp;
			timerhw->tcrr = tmp;
			tmp = tmp + ((V_OSCK/pwmfrq)/100) * bright;
			timerhw->tmar = tmp;
			timerhw->tclr = (TCLR_PT | (2 << TCLR_TRG_SHIFT) |
					TCLR_CE | TCLR_AR | TCLR_ST);
		} else {
			puts("invalid pwmfrq in env/dtb! skip PWM-setup.\n");
		}
		break;
	default:
		puts("no suitable backlightdriver in env/dtb!\n");
		break;
	}
#endif
}
#elif CONFIG_SPL_BUILD
#else
#error "LCD-support with a suitable FB-Driver is mandatory !"
#endif /* CONFIG_LCD */
