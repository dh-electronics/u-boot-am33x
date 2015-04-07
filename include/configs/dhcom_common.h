#ifndef __DHCOM_COMMON_H
#define __DHCOM_COMMON_H

#ifdef CONFIG_DHCOM
 
/* default settings */
#define DEFAULT_SETTINGS_BLOCK_LENGTH       0x2C        /* 44 Byte */
#define DEFAULT_SETTINGS_DISPLAY_ID         0x01
#define DEFAULT_SETTINGS_Y_RESOLUTION       272
#define DEFAULT_SETTINGS_X_RESOLUTION       480
#define DEFAULT_SETTINGS_LCD_CONFIG_FLAGS   0x03E7      /* includes GPIO G for BE enable */

#define DEFAULT_SETTINGS_PIXEL_CLOCK        11100
#define DEFAULT_SETTINGS_V_PULSE_WIDTH      11
#define DEFAULT_SETTINGS_H_PULSE_WIDTH      42
#define DEFAULT_SETTINGS_H_BACK_PORCH       2
#define DEFAULT_SETTINGS_H_FRONT_PORCH      3
#define DEFAULT_SETTINGS_V_BACK_PORCH       2
#define DEFAULT_SETTINGS_V_FRONT_PORCH      3
#define DEFAULT_SETTINGS_AC_BIAS_TRANS      0
#define DEFAULT_SETTINGS_AC_BIAS_FREQ       0
#define DEFAULT_SETTINGS_DATALINES          24

#define DEFAULT_SETTINGS_GPIO_DIRECTION     0xFFFFFFFF  /* set direction to input */
#define DEFAULT_SETTINGS_GPIO_STATE         0x00000000  /* set states to low */
        
#define DEFAULT_SETTINGS_HW_CONFIG_FLAGS    0x027E /* SILENT_MODE = off , UPDATE_DEV = all , USE_DA_EEPROM_SETTINGS = on */     

#define DEFAULT_BACKLIGHT_PWM                57 /* DHCOM GPIO_PWM - this is fix and not to change with settings.bin functionality */

/* dhcom gpios */
#define DHCOM_GPIO_A   (0+20)
#define DHCOM_GPIO_B   (3*32+19)
#define DHCOM_GPIO_C   (3*32+20)
#define DHCOM_GPIO_D   (2*32+1)
#define DHCOM_GPIO_E   (0+18)
#define DHCOM_GPIO_F   (3*32+16)
#define DHCOM_GPIO_G   (3*32+15)
#define DHCOM_GPIO_H   (0+7)
#define DHCOM_GPIO_I   (3*32+17)
#define DHCOM_GPIO_J   (3*32+21)
#define DHCOM_GPIO_K   (1*32+25)
#define DHCOM_GPIO_L   (1*32+24)
#define DHCOM_GPIO_M   (1*32+23)
#define DHCOM_GPIO_N   (1*32+22)
#define DHCOM_GPIO_O   (1*32+19)
#define DHCOM_GPIO_P   (1*32+18)
#define DHCOM_GPIO_Q   (1*32+17)
/* not available on dhcom am335x */
#undef DHCOM_GPIO_R   
#undef DHCOM_GPIO_S
#undef DHCOM_GPIO_T
#undef DHCOM_GPIO_U
#undef DHCOM_GPIO_V
#undef DHCOM_GPIO_W

/* additional u-boot commands */
#define CONFIG_CMD_SETTINGS_INFO
#define CONFIG_CMD_DHCOM_UPDATE

#define UPDATE_DHUPDATE_INI_SDRAM_ADDRESS "80000000"  /* should equal os base */

#endif

#endif
