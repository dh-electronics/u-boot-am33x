/*
 * File: cmd_settings_info.c
 *
 * (C) Copyright 2011-2012
 * Andreas Geisreiter , DH electronics GmbH , ageisreiter@dh-electronics.de
 * Ludwig Zenz, DH electronics GmbH , lzenz@dh-electronics.de
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 *  Version:  1.1
 *
 *  Abstract: This file contains the source code for the "lcdinfo" command line function.
 *            This function lists the act. i.MX25 display controller settings.
 *
 *  Notes:
 *      Created:                May 10, 2011 by Andreas Geisreiter (ageisreiter@dh-electronics.de)
 *      Modified:               Jan 25, 2012 by Ludwig Zenz (lzenz@dh-electronics.de)
 */

#include <common.h>
#include <command.h>
#include <lcd.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/errno.h>

DECLARE_GLOBAL_DATA_PTR;

int do_settings_info ( cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
        volatile settingsinfo_t *gsb = &gd->dh_board_settings; /* get pointer to global settings block */

        if (gsb->wValidationID != 0) {
                printf("  VALIDATION_ID:    \"%c%c\"\n",
                                (char)gsb->wValidationID,
                                (char)(gsb->wValidationID >> 8));
        } else {
                printf("  VALIDATION_ID:    0x0000\n");
        }
        printf("  DISPLAY_ID:       0x%02x\n", gsb->cDisplayID);
        printf("  LENGTH:           0x%02x\n", gsb->cLength);
        printf("  X_RESOLUTION:     %d pixel\n", gsb->wXResolution);
        printf("  Y_RESOLUTION:     %d pixel\n", gsb->wYResolution);
        printf("  PIXEL_CLOCK:      %d kHz\n", gsb->wPixelClock);
        printf("  LCD_CONFIG_FLAGS: 0x%x\n", gsb->wLCDConfigFlags);
        printf("        IVS:        0x%x\n", (gsb->wLCDConfigFlags & SETTINGS_LCD_IVS_FLAG));
        printf("        IHS:        0x%x\n", ((gsb->wLCDConfigFlags & SETTINGS_LCD_IHS_FLAG) >> 1));
        printf("        IPC:        0x%x\n", ((gsb->wLCDConfigFlags & SETTINGS_LCD_IPC_FLAG) >> 2));
        printf("        IOE:        0x%x\n", ((gsb->wLCDConfigFlags & SETTINGS_LCD_IOE_FLAG) >> 3));
        printf("        IDATA:      0x%x\n", ((gsb->wLCDConfigFlags & SETTINGS_LCD_IDATA_FLAG) >> 4));
        printf("        ACT_PAS:    0x%x\n", ((gsb->wLCDConfigFlags & SETTINGS_LCD_ACT_PAS_FLAG) >> 5));
        printf("        PWM_POL:    0x%x\n", ((gsb->wLCDConfigFlags & SETTINGS_LCD_PWM_POL_FLAG) >> 6));
        printf("        BL_EN_GPIO: 0x%x\n", ((gsb->wLCDConfigFlags & SETTINGS_LCD_BL_EN_GPIO_FLAG) >> 7));
        printf("        IBL:        0x%x\n", ((gsb->wLCDConfigFlags & SETTINGS_LCD_IBL_FLAG) >> 11));
        if(gsb->wValidationID == 0x3256) { // valid marker "V2" = 0x3256
                printf("        BL_ON:      0x%x\n", ((gsb->wLCDConfigFlags & SETTINGS_LCD_BL_ON_FLAG) >> 12));
                printf("        DI_TYPE:    0x%x\n", ((gsb->wLCDConfigFlags & SETTINGS_LCD_DI_TYPE_FLAG) >> 13));
                printf("        NEXT_DI:    0x%x\n", ((gsb->wLCDConfigFlags & SETTINGS_LCD_NEXT_DI_FLAG) >> 16));
        }
        printf("  HSW:              %d pixel clocks\n", gsb->wHPulseWidth);
        printf("  VSW:              %d line clocks\n", gsb->wVPulseWidth);
        printf("  HFP:              %d pixel clocks\n", gsb->wHFrontPorch);
        printf("  HBP:              %d pixel clocks\n", gsb->wHBackPorch);
        printf("  VFP:              %d line clocks\n", gsb->wVFrontPorch);
        printf("  VBP:              %d line clocks\n", gsb->wVBackPorch);
        printf("  DATALINES:        %d\n", gsb->cDatalines);
        printf("  ACB:              %d\n", gsb->cACBiasFreq);
        printf("  ACBI:             %d\n\n", gsb->cACBiasTrans);

        printf("  GPIO_DIR:         0x%08x\n", gsb->wGPIODir);
        printf("  GPIO_STATE:       0x%08x\n", gsb->wGPIOState);
        printf("  HW_CONFIG_FLAGS:  0x%04x\n\n", gsb->wHWConfigFlags);

        return 0;
}

U_BOOT_CMD(
        settings,   1,   1,     do_settings_info,
        "shows the DHCOM settings",
        "\n"
);

