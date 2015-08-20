/*
 * File: cmd_DHCOMupdate.c
 *
 * (C) Copyright 2011-2012
 * Andreas Geisreiter, DH electronics GmbH , ageisreiter@dh-electronics.de
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

/* ------------------------------------------------------------------------------
 *
 * Abstract: Command line function to update the flash content from storage device.
 *
 * Notes:
 *     Created by:  Andreas Geisreiter (ageisreiter@dh-electronics.de)
 *     Created:     January 03, 2012
 *
 * Description:
 *     The following command line calls are possible:
 *     1. "update auto":
 *        Is called after the start of the bootloader with the env variable "bootcmd". This function call is
 *        used for automatic update at the system start. The function tries to update the flash content via
 *        the DHupdate.ini file. The update mechanism is searching for the update files on the specified
 *        storage devices. The update devices must be defined at the settings block. With the DHupdate.ini
 *        file you have the possibility to run more then one updates, e.g. OS image and settings block.
 *
 *     2. "update":
 *        Is used form the command line. This function call tries to update the flash content via the
 *        DHupdate.ini file. In contrast to 1. the update mechanism is searching on every available storage
 *        device for the update files.
 *
 *     3. "update <type> [filename]":
 *        Is used form the command line. This function call doesn't use the DHupdate.ini file. It
 *        allows only one update and the update mechanism is searching on every available storage device for
 *        the update files. With the <type> parameter you have to specify the update type.
 *
 *     Possible Updates:
 *     Argument = '1' ==> Update only WinCE Flash Image (nk.bin or nk.gz)
 *     Argument = '2' ==> Update only Linux Flash Image
 *     Argument = '3' ==> Update Bootloader Flash Image
 *     Argument = '4' ==> Update splash.bmp (Start Screen Picture)
 *     Argument = '5' ==> Update settings.bin file
 *     Argument = '6' ==> Refresh DH settings and LCD controller
 *     Argument = '7' ==> Display adapter EEPROM settings update
 *     Argument = '8' ==> Reset Board
 *     Argument = '9' ==> Update rootfs in NAND-Flash
 */

#include <config.h>
#include <common.h>
#include <command.h>
#include <i2c.h>
#include <linux/ctype.h>
#include <asm/io.h>
#include <dh_settings.h>
#include <dh_update.h>

#if defined(CONFIG_CMD_DHCOM_UPDATE)

DECLARE_GLOBAL_DATA_PTR;

// ===============================================================================================================
// Defines
// ===============================================================================================================
#define UPDATE_INI_CMDLINE      0 // = Command line Update with more then 1 arguments (e.g. "update wince dh_nk.gz")
#define UPDATE_INI_FILE         1 // = DHupdate.ini file Update ("update auto" or command line call "update")
#define UPDATE_INI_ERROR        2 // = error

// ===============================================================================================================
// Extern defined variables
// ===============================================================================================================

// ===============================================================================================================
// Extern defined functions
// ===============================================================================================================
//extern int do_load_wrapper (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int do_spi_flash(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]); /* cmd_sf.c */
extern int do_ls_wrapper(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]); /* cmd_fs.c */
extern int do_load_wrapper(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]); /* cmd_fs.c */
extern int do_mem_cp(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]); /* cmd_mem.c */
extern int do_source(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]); /* cmd_source.c */
extern int do_env_set(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]); /* cmd_nvedit.c */
extern int do_mmcops(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]); /* cmd_mmc.c */
extern int do_usb(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]); /* cmd_usb.c */
extern int do_bmp (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int DHCOMUpdateLED_Init(updateinfo_t *p_stDHupdateINI);
extern void DHCOMUpdateDelayMs(unsigned long msec);
extern void DHCOMUpdateLED_SetHigh(void);
extern void DHCOMUpdateLED_SetLow(void);

void CopyAddressStringToCharArray(char *p_cCharArray, char *p_cPointer)
{
        int i,j;

        for(i = 0, j = 0; i < 8; i++, j++) {
                // Check if string contains "0x"
                if(p_cPointer[j] == 'x') {
                        i = 0; /* reset write index*/
                        j++;
                }
                p_cCharArray[i] = p_cPointer[j];
        }
        p_cCharArray[i] = '\0';
}

//------------------------------------------------------------------------------
//
//  Function:  update_flash_content
//
//  Update Flash content on the specified address.
//
//  Commit values:  - ulOffset    = Flash Offset address
//                  - ulSDRAMBufferAddress  = SDRAM Buffer address
//                  - ulBlocks              = Number of block that should be updated
//                  - ulFlashBlockSize      = Flash block size
//
//  Return value:   0 = No error
//                  1 = Flash erase error (the error block number is specified from bit 3 to bit 31)
//                  2 = Flash write error (the error block number is specified from bit 3 to bit 31)
//
int update_flash_content (unsigned long ulOffset, unsigned long ulSDRAMBufferAddress, unsigned long ulBlocks, unsigned long ulFlashBlockSize)
{
        char *p_cEraseFlashBlock[4] = {"sf", "erase", "12345671", "12345672"};
        char *p_cWriteFlashBlock[5] = {"sf", "write", "12345673", "12345674", "12345675"};

        unsigned long i;
        int ret_value = 0;

        // Set FLash Block size
        sprintf (p_cEraseFlashBlock[3], "%08x", (unsigned int)ulFlashBlockSize);
        sprintf (p_cWriteFlashBlock[4], "%08x", (unsigned int)ulFlashBlockSize);

        DISABLE_PRINTF()
        run_command("sf probe", 0);
        ENABLE_PRINTF()

        for(i = 0; i < ulBlocks; i++) {

                if((i%50) == 0) {
                        printf ("\n                         ");
                }

                // Set address/offset
                sprintf (p_cEraseFlashBlock[2], "%08x", (unsigned int)(ulOffset + i * ulFlashBlockSize));

                sprintf (p_cWriteFlashBlock[2], "%08x", (unsigned int)(ulSDRAMBufferAddress));
                sprintf (p_cWriteFlashBlock[3], "%08x", (unsigned int)(ulOffset + i * ulFlashBlockSize));

                // Erase flash block
                printf ("e");

                DISABLE_PRINTF()
                ret_value = do_spi_flash(NULL, 0, 4, p_cEraseFlashBlock);
                ENABLE_PRINTF()

                // Exit on Erase ERROR
                if(ret_value != 0) {
                        ret_value = (i << 2) | 1;
                        return ret_value;
                }

                // Write flash block
                printf ("\bw");
                DISABLE_PRINTF()
                ret_value = do_spi_flash(NULL, 0, 5, p_cWriteFlashBlock);
                ENABLE_PRINTF()

                // Exit on Write ERROR
                if(ret_value != 0) {
                        ret_value = (i << 2) | 2;
                        return ret_value;
                }

                // Set SDRAM Buffer address
                ulSDRAMBufferAddress += ulFlashBlockSize;

                printf ("\bd");
        }

        printf ("]");
        return 0;
}

//------------------------------------------------------------------------------
//
//  Function:  update_eeprom_content
//
//  Update EEProm content on the specified address.
//
//  Commit values:  - ulEepromRegisterAddress = Eeprom register offset
//                  - ulSDRAMBufferAddress  = SDRAM Buffer address
//                  - ulBytes               = Number of bytes that should be updated
//
//  Return value:   0 = No error
//                  1 = EEPROM write error
//
int update_eeprom_content (unsigned long ulEepromRegisterAddress, unsigned long ulSDRAMBufferAddress, unsigned long ulBytes)
{
        unsigned int i,j;
        int ret_value = 0;
        uchar ucBuffer;

        /* Set i2c driver to use i2c bus 0  */
        DISABLE_PRINTF()
        ret_value = run_command ("i2c dev 0", 0);
        ENABLE_PRINTF()
        if (ret_value != 0)
        {
                return 1;
        }

        for(j = 1, i = 0; i < ulBytes; j++, i++)
        {
                if(j == 51)
                {
                        printf ("\n                         ");
                        j = 1;
                }

                // Write eeprom content
                printf ("w");
                DISABLE_PRINTF()
                ret_value = i2c_write(DISPLAY_ADAPTER_EEPROM_ADDR, i, 1, (uchar*)(ulSDRAMBufferAddress+i), 1);
                DHCOMUpdateDelayMs(8);
                ENABLE_PRINTF()

                // Exit on Write ERROR
                if(ret_value != 0)
                {

                        /* Set i2c driver back to use i2c bus 2 */
                        DISABLE_PRINTF()
                        ret_value = run_command ("i2c dev 2", 0);
                        ENABLE_PRINTF()
                        if (ret_value != 0)
                        {
                                return 1;
                        }


                        return 1;
                }

                printf ("\bd");
        }

        // Verify EEPROM data
        for(i = 0; i < ulBytes; i++)
        {
                DISABLE_PRINTF()
                ret_value = i2c_read(DISPLAY_ADAPTER_EEPROM_ADDR, i, 1, &ucBuffer, 1);
                ENABLE_PRINTF()

                // Exit on Write ERROR
                if((ret_value != 0) || (ucBuffer != *(uchar*)(ulSDRAMBufferAddress+i)))
                {

                        /* Set i2c driver back to use i2c bus 2 */
                        DISABLE_PRINTF()
                        ret_value = run_command ("i2c dev 2", 0);
                        ENABLE_PRINTF()
                        if (ret_value != 0)
                        {
                                return 1;
                        }

                        return 1;
                }
        }

        printf ("]");

        /* Set i2c driver back to use i2c bus 2 */
        DISABLE_PRINTF()
        ret_value = run_command ("i2c dev 2", 0);
        ENABLE_PRINTF()
        if (ret_value != 0)
        {
                return 1;
        }

        return 0;
}

//------------------------------------------------------------------------------
//
//  Function:  UpdateGlobalDataDHSettings
//
//  Update the DHsettings global data structure from ram
//
//  Commit values:  - addr = Address of the new settings file.
//
//  Return value:   0 = No error
//                  1 = error
//
int UpdateGlobalDataDHSettings (ulong addr)
{
        // settings.bin file Valid Mask should be "DH" = 0x4844
        if((((*(ulong *)addr) >> 16) & 0xFFFF) == 0x4844)
        {
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

                gd->dh_board_settings.wGPIODir = readl(addr+32);
                gd->dh_board_settings.wGPIOState = readl(addr+36);

                gd->dh_board_settings.wHWConfigFlags = (readl(addr+40) & 0xFFFF);

                return 0;
        }
        return 1; /* no valid settings.bin available */
}

//------------------------------------------------------------------------------
//
//  Function:  check_imagesize
//
//  Update Flash content on the specified address.
//
//  Commit values:  - ulFilesize            = Size of the image to write
//                  - ulFlashPartitionSize  = available partitionsize
//                  - cUpdatetype           = Identifier for updatetype (Customizing Error-Messages)
//
//  Return value:   0 = No error
//                  1 = image is to large
//
int check_imagesize (unsigned long ulFilesize, unsigned long ulFlashPartitionSize, char cUpdatetype, char* pcErrorString )
{
        if (ulFilesize > ulFlashPartitionSize)
        {
                switch (cUpdatetype)
                {
                case BOOTLOADER_FLASH_UPDATE:
                        sprintf (pcErrorString, "\n==> Update ERROR: u-boot image to large: max %li bytes\n", ulFlashPartitionSize);
                        break;
                case EEPROM_SETTINGS_UPDATE:
                        sprintf (pcErrorString, "\n==> Update ERROR: Eeprom.bin to large: max %li bytes\n", ulFlashPartitionSize);
                        break;
                default:
                        sprintf (pcErrorString, "\n==> Update ERROR: Image to large: max %li bytes\n", ulFlashPartitionSize);
                }

                return -1;
        }

        return 0; /* no error*/
}

//------------------------------------------------------------------------------
//
//  Function:  ReadDHupdateFile
//
//  Read DHupdate.ini file content.
//
//  Commit values:  - *p_stDHupdateINI = Pointer to DHupdateINI struct
//                  - ulDHUpdateIniSDRAMAddress = DHupdate.ini file SDRAM address
//
//  Return value:   0 = No error
//                  1 = error
//
int ReadDHupdateFile(updateinfo_t *p_stDHupdateINI, unsigned long ulDHUpdateIniSDRAMAddress)
{
        void *p_vFilePointer = (void*)ulDHUpdateIniSDRAMAddress;    // Pointer to DHupdate.ini file
        int iPositionPointer = 0;                                   // Act. position in DHupdate.ini file
        int iPositionPointerMemorize = 0;                           // Old position in DHupdate.ini file
        int iDifference = 0;                                        // differenc between act. and old position
        int iSpecialFilename = 0;
        char *p_cCompareStringDisplay = {"display\0"};
        char *p_cCompareStringLED = {"led\0"};
        char *p_cCompareStringUpdate = {"update\0"};
        char *p_cCompareStringEnd = {"end\0"};

        // Set to invalid
        p_stDHupdateINI->iDisplayInfo = 0;
        p_stDHupdateINI->iLedInfo = 0;
        p_stDHupdateINI->iUpdateCounter = 0;

        // Search in DHupdate.ini file for valid mask
        for(iPositionPointer = 0; *(char*)(p_vFilePointer + iPositionPointer) != '\n'; iPositionPointer++);
        *(char*)(p_vFilePointer + iPositionPointer) = '\0';
        p_stDHupdateINI->p_cValidMarker = (char*)p_vFilePointer;

        iPositionPointer++;
        // Search for '['']' content
        while(*(char*)(p_vFilePointer + iPositionPointer) == '[')
        {
                iPositionPointer++;
                iPositionPointerMemorize = iPositionPointer;
                while(*(char*)(p_vFilePointer + iPositionPointer) != ']')
                {
                        iPositionPointer++;
                }
                *(char*)(p_vFilePointer + iPositionPointer) = '\0';
                iDifference = iPositionPointer - iPositionPointerMemorize;

                if(!(memcmp ( (char*)(p_vFilePointer+iPositionPointer-iDifference), p_cCompareStringDisplay, iDifference)))
                {
                        // **********************************************************
                        // ***************** Update Block [display] *****************
                        // **********************************************************

                        p_stDHupdateINI->iDisplayInfo = 1;
                        // Progress Bmp
                        iPositionPointer+=2; // +2, because ']' and '\n'
                        iPositionPointerMemorize = iPositionPointer;
                        while(*(char*)(p_vFilePointer + iPositionPointer) != '\n')
                        {
                                iPositionPointer++;
                        }
                        *(char*)(p_vFilePointer + iPositionPointer) = '\0';
                        p_stDHupdateINI->p_cFileNameProgressBmp = (char*)(p_vFilePointer+iPositionPointerMemorize);
                        // Ok Bmp
                        iPositionPointer++;
                        iPositionPointerMemorize = iPositionPointer;
                        if(*(char*)(p_vFilePointer + iPositionPointer) == '[')
                        {
                                // DHupdate.ini display file missing
                                return 1;
                        }
                        while(*(char*)(p_vFilePointer + iPositionPointer) != '\n')
                        {
                                iPositionPointer++;
                        }
                        *(char*)(p_vFilePointer + iPositionPointer) = '\0';
                        p_stDHupdateINI->p_cFileNameOkBmp = (char*)(p_vFilePointer+iPositionPointerMemorize);
                        // Error Bmp
                        iPositionPointer++;
                        iPositionPointerMemorize = iPositionPointer;
                        if(*(char*)(p_vFilePointer + iPositionPointer) == '[')
                        {
                                // DHupdate.ini display file missing
                                return 1;
                        }
                        while(*(char*)(p_vFilePointer + iPositionPointer) != '\n')
                        {
                                iPositionPointer++;
                        }
                        *(char*)(p_vFilePointer + iPositionPointer) = '\0';
                        p_stDHupdateINI->p_cFileNameErrorBmp = (char*)(p_vFilePointer+iPositionPointerMemorize);
                        iPositionPointer++;
                }
                else if(!(memcmp ( (char*)(p_vFilePointer+iPositionPointer-iDifference), p_cCompareStringLED, iDifference)))
                {
                        // **********************************************************
                        // ******************* Update Block [led] *******************
                        // **********************************************************
                        p_stDHupdateINI->iLedInfo = 1;
                        // Update GPIO
                        iPositionPointer+=2; // +2, because ']' and '\n'
                        iPositionPointerMemorize = iPositionPointer;
                        while((*(char*)(p_vFilePointer + iPositionPointer) != '\n') && (*(char*)(p_vFilePointer + iPositionPointer) != ' '))
                        {
                                iPositionPointer++;
                        }

                        if(*(char*)(p_vFilePointer + iPositionPointer) == '\n')
                        {
                                // Active state is missing --> Don't set Update GPIO, because we don't know the active state of the connected LED
                                p_stDHupdateINI->iLedInfo = 0;
                                iPositionPointer++;
                        }
                        else
                        {
                                // GPIO name
                                *(char*)(p_vFilePointer + iPositionPointer) = '\0';
                                p_stDHupdateINI->p_cUpdateGpioName = (char*)(p_vFilePointer+iPositionPointerMemorize);
                                iPositionPointer++;

                                // GPIO Active state
                                iPositionPointerMemorize = iPositionPointer;
                                while(*(char*)(p_vFilePointer + iPositionPointer) != '\n')
                                {
                                        iPositionPointer++;
                                }

                                *(char*)(p_vFilePointer + iPositionPointer) = '\0';

                                if (strcmp((char*)(p_vFilePointer+iPositionPointerMemorize), "high") == 0)
                                {
                                        p_stDHupdateINI->iUpdateGpioActiveState = 1;
                                }
                                else if (strcmp((char*)(p_vFilePointer+iPositionPointerMemorize), "low") == 0)
                                {
                                        p_stDHupdateINI->iUpdateGpioActiveState = 0;
                                }
                                else
                                {
                                        // Wrong active state argument
                                        p_stDHupdateINI->iLedInfo = 0;
                                }

                                iPositionPointer++;
                        }
                }
                else if(!(memcmp ( (char*)(p_vFilePointer+iPositionPointer-iDifference), p_cCompareStringUpdate, iDifference)))
                {
                        // **********************************************************
                        // ***************** Update Block [update] ******************
                        // **********************************************************
                        iPositionPointer+=2; // +2, because ']' and '\n'

                        while(*(char*)(p_vFilePointer + iPositionPointer) != '[')
                        {
                                if(p_stDHupdateINI->iUpdateCounter >= DHUPDATEINI_MAX_UPDATES)
                                {
                                        // Too much update arguments in DHupdate.ini file
                                        return 1;
                                }

                                p_stDHupdateINI->iUpdateCounter++;
                                // Update Type
                                iPositionPointerMemorize = iPositionPointer;

                                // read text until '\n' or ' '
                                while((*(char*)(p_vFilePointer + iPositionPointer) != '\n') && (*(char*)(p_vFilePointer + iPositionPointer) != ' '))
                                {
                                        iPositionPointer++;
                                }

                                // Only if ' ' after update type a custom filename should be available
                                if(*(char*)(p_vFilePointer + iPositionPointer) == ' ')
                                {
                                        iSpecialFilename = 1;
                                }
                                else
                                {
                                        iSpecialFilename = 0;
                                }

                                *(char*)(p_vFilePointer + iPositionPointer) = '\0'; // Set '\0' to mark end of update-type string

                                // set pointer to update-type string in stDHUpdateInfo array
                                p_stDHupdateINI->stDHUpdateInfo[p_stDHupdateINI->iUpdateCounter - 1].p_cUpdateType = (char*)(p_vFilePointer+iPositionPointerMemorize);
                                iPositionPointer++;

                                if(iSpecialFilename)
                                {
                                        // Update Type
                                        iPositionPointerMemorize = iPositionPointer;
                                        while(*(char*)(p_vFilePointer + iPositionPointer) != '\n')
                                        {
                                                iPositionPointer++;
                                        }
                                        *(char*)(p_vFilePointer + iPositionPointer) = '\0'; // Set '\0' to mark end of filename string

                                        // set pointer to filename string in stDHUpdateInfo array
                                        p_stDHupdateINI->stDHUpdateInfo[p_stDHupdateINI->iUpdateCounter - 1].p_cFilename = (char*)(p_vFilePointer+iPositionPointerMemorize);
                                        iPositionPointer++;
                                }
                                else
                                {
                                        p_stDHupdateINI->stDHUpdateInfo[p_stDHupdateINI->iUpdateCounter - 1].p_cFilename = NULL;
                                }
                        }
                }
                else if(!(memcmp ( (char*)(p_vFilePointer+iPositionPointer-iDifference), p_cCompareStringEnd, iDifference)))
                {
                        return 0;
                }
                else
                {
                        return 1;
                }
        }

        return 1;
}

//------------------------------------------------------------------------------
//
//  Function:  ShowBitmap
//
//  Shows the specified Update Bitmap on the Display
//
//  Commit values:  - *p_stDHupdateINI = Pointer to DHupdateINI struct
//                  - eBitmapType = enum to specify Bitmap (PROGRESS_BITMAP, END_BITMAP and ERROR_BITMAP)
//                  - p_cStorageDevice = Specify the act. storage device (e.g. "usb")
//                  - p_cDevicePartitionNumber = Specify the act. partition number of the storage device
//
//  Return value:   0 = No error
//                  1 = error
//
//
int ShowBitmap(updateinfo_t *p_stDHupdateINI, enum BitmapTypeEnum eBitmapType, char *p_cStorageDevice, char *p_cDevicePartitionNumber)
{
        char cENVSDRAMBufferAddress[9];

        CopyAddressStringToCharArray(&cENVSDRAMBufferAddress[0], getenv ("loadaddr"));

        char *p_cLoadBmpToSDRAM[5]      = {"load","","",cENVSDRAMBufferAddress,""};
        char *p_cDisplayBmpOnScreen[5]  = {"bmp","display",cENVSDRAMBufferAddress,"32767","32767"};
        int ret_value                   = 0;

        if(p_stDHupdateINI->iDisplayInfo != 1) {
                /* update bmp's not specified in DHupdate.ini, return ... */
                return 0;
        }

        // Set current device (mmc or usb)
        p_cLoadBmpToSDRAM[1] = p_cStorageDevice;

        // Set current device number and partition
        p_cLoadBmpToSDRAM[2] = p_cDevicePartitionNumber;

        switch(eBitmapType)
        {
        case PROGRESS_BITMAP:
                // Set "progress bmp" filename from DHupdate.ini file
                p_cLoadBmpToSDRAM[4] = p_stDHupdateINI->p_cFileNameProgressBmp;
                break;
        case END_BITMAP:
                // Set "done bmp" filename from DHupdate.ini file
                p_cLoadBmpToSDRAM[4] = p_stDHupdateINI->p_cFileNameOkBmp;
                break;
        case ERROR_BITMAP:
                // Set "error bmp" filename from DHupdate.ini file
                p_cLoadBmpToSDRAM[4] = p_stDHupdateINI->p_cFileNameErrorBmp;
                break;
        default:
                return 1;
                break;
        }

        // Load bitmap from MMC/SD - Card to SDRAM.
        DISABLE_PRINTF()
        ret_value = do_load_wrapper(NULL, 0, 5, p_cLoadBmpToSDRAM);
        ENABLE_PRINTF()

        if(ret_value == 0) {
                do_bmp(NULL, 0, 5, p_cDisplayBmpOnScreen);
                return 0;
        }

        return 1;
}

//------------------------------------------------------------------------------
//
//  Function:  ActivateUpdateGPIO
//
//  Activate the Update GPIO
//
//  Commit values:  - *p_stDHupdateINI = Pointer to DHupdateINI struct
//
//  Return value:   -
//
void ActivateUpdateGPIO(updateinfo_t *p_stDHupdateINI)
{
        if(p_stDHupdateINI->iUpdateGpioActiveState == 1)
                DHCOMUpdateLED_SetHigh();
        else
                DHCOMUpdateLED_SetLow();
}

//------------------------------------------------------------------------------
//
//  Function:  DeactivateUpdateGPIO
//
//  Deactivate the Update GPIO
//
//  Commit values:  - *p_stDHupdateINI = Pointer to DHupdateINI struct
//
//  Return value:   -
//
void DeactivateUpdateGPIO(updateinfo_t *p_stDHupdateINI)
{
        if(p_stDHupdateINI->iUpdateGpioActiveState == 1)
                DHCOMUpdateLED_SetLow();
        else
                DHCOMUpdateLED_SetHigh();
}

//------------------------------------------------------------------------------
//
//  Function:  UpdateGPIOBlinkInterval
//
//  Let the update LED blink.
//
//  Commit values:  - *p_stDHupdateINI = Pointer to DHupdateINI struct
//                  - iNumber = Blinking number
//
//  Return value:   -
//
void UpdateGPIOBlinkInterval(updateinfo_t *p_stDHupdateINI, int iNumber)
{
        for( ; iNumber > 0; iNumber--) {
                DHCOMUpdateDelayMs(250);
                ActivateUpdateGPIO(p_stDHupdateINI);
                DHCOMUpdateDelayMs(250);
                DeactivateUpdateGPIO(p_stDHupdateINI);
        }
}

//------------------------------------------------------------------------------
//
//  Function:  ShowUpdateError
//
//  Shows the specified Update Error String on the console, loads error bitmap
//  and sets the LED GPIO.
//
//  Commit values:  - *p_stDHupdateINI = Pointer to DHupdateINI struct
//                  - p_cErrorStringPointer = pointer to the error string
//                  - eDHCOMUpdateError = Specifies the error for the LED blink algorithm
//                  - update_ini = Specifies if Update is started
//                                                form console or from DHupdate.ini file
//                  - p_cStorageDevice = Specify the act. storage device (e.g. "usb")
//                  - p_cDevicePartitionNumber = Specify the act. partition number of the storage device
//
//  Return value:   -
//
void ShowUpdateError(updateinfo_t *p_stDHupdateINI, char *p_cErrorStringPointer, enum UpdateErrorEnum eDHCOMUpdateError, int update_ini, char *p_cStorageDevice, char *p_cDevicePartitionNumber)
{
        int led_blink_code = 0;

        printf ("%s", p_cErrorStringPointer);

        if(update_ini != UPDATE_INI_FILE) {
                /* update visualisation with *.bmp and gpio is
                 * only supported in DHupdate.ini file mode */
                return;
        }

        if(ShowBitmap(p_stDHupdateINI, ERROR_BITMAP, p_cStorageDevice, p_cDevicePartitionNumber) != 0) {
                printf ("\n--> Update INFO: Error bitmap %s not found\n", p_stDHupdateINI->p_cFileNameErrorBmp);
        }

        if(p_stDHupdateINI->iLedInfo != 1) {
                /* no led gpio in DHupdate.ini file */
                return;
        }

        /* translate error code to led blink code */
        switch(eDHCOMUpdateError)
        {
        case DHUPDATE_INI_ERROR:
                led_blink_code = 1;
                break;
        case FILE_NOT_FOUND_ERROR:
                led_blink_code = 2;
                break;
        case FLASH_ERROR:
                led_blink_code = 3;
                break;
        case IMAGE_TYPE_ERROR:
                led_blink_code = 4;
                break;
        case INVALID_FILE_ERROR:
                led_blink_code = 5;
                break;
        case IMAGE_SIZE_ERROR:
                led_blink_code = 6;
                break;
        case CANT_LOAD_UPDATE_KERNEL:
                led_blink_code = 7;
                break;
        default:
                /* No or a unknown error */
                return;
        }

        /* do the led blinking */
        DeactivateUpdateGPIO(p_stDHupdateINI);
        while(1) {
                UpdateGPIOBlinkInterval(p_stDHupdateINI, 1);
                DHCOMUpdateDelayMs(1750);
        }

        return;
}

//------------------------------------------------------------------------------
//
//  Function:  DHCOMupdate
//
//  DHCOM U-Boot Update function
//
//  Commit values:  - *cmdtp = Command Type, used for cmd_usage()
//                  - argc = Number of command line arguments
//                  - *argv[] = Command line argument strings
//                  - *p_stDHupdateINI = Pointer to DHupdateINI struct
//                  - p_cStorageDevice = Specify the act. storage device (e.g. "usb")
//                  - p_cDevicePartitionNumber = Specify the act. partition number of the storage device
//
//  Return value:   0 = No error
//                  1 = error
//
int DHCOMupdate (cmd_tbl_t *cmdtp, int argc, char * const argv[], updateinfo_t *p_stDHupdateINI, char *p_cStorageDevice, char *p_cDevicePartitionNumber)
{
        char cENVSDRAMBufferAddress[9];
        CopyAddressStringToCharArray(&cENVSDRAMBufferAddress[0], getenv ("loadaddr"));

        char cDHUpdateIniSDRAMAddress[9]        = {UPDATE_DHUPDATE_INI_SDRAM_ADDRESS};

        /* get address of copy-buffer */
        unsigned long ulSDRAMBufferAddress      = simple_strtoul (cENVSDRAMBufferAddress, NULL, 16);

        int update_ini = UPDATE_INI_FILE;

        /* get u-boot flash offset and partition size */
        unsigned long ulBootloaderOffset = CONFIG_SYS_SPI_U_BOOT_OFFS;
        unsigned long ulBootloaderFlashPartitionSize = CONFIG_ENV_OFFSET;
        unsigned long ulFilesize;
        unsigned long ulBlocks;
        unsigned long ulFlashBlockSize = CONFIG_ENV_SECT_SIZE;

        char cUpdateArgument = 0;
        char cErrorString[100];
        int iRefreshStringFound = 0;
        int iUpdateLoopCounter = 0;
        int iLoadUpdateKernel = 0;
        int i = 0;

        void *p_vUpdateArgument;
        char *cmd, *file_name;
        char p_cCompareString[] = "##DHCOMupdate##";
        char *p_cLoadDHUpdateIniToSDRAM[5]      = {"load","","",cDHUpdateIniSDRAMAddress,"DHupdate.ini"};
        char *p_cLoadUBootBinToSDRAM[5]         = {"load","","",cENVSDRAMBufferAddress,"u-boot.img"};
        char *p_cLoadEepromBinToSDRAM[5]        = {"load","","",cENVSDRAMBufferAddress,"eeprom.bin"};
        char *p_cLoadScriptBinToSDRAM[5]        = {"load","","",cENVSDRAMBufferAddress,"script.bin"};
        char *p_cRunScript[2]                   = {"source",cENVSDRAMBufferAddress};

        int ret_value = 0;

        // Set current device (mmc or usb)
        p_cLoadDHUpdateIniToSDRAM[1] = p_cStorageDevice;
        p_cLoadUBootBinToSDRAM[1] = p_cStorageDevice;
        p_cLoadEepromBinToSDRAM[1] = p_cStorageDevice;
        p_cLoadScriptBinToSDRAM[1] = p_cStorageDevice;

        // Set current device number and partition
        p_cLoadDHUpdateIniToSDRAM[2] = p_cDevicePartitionNumber;
        p_cLoadUBootBinToSDRAM[2] = p_cDevicePartitionNumber;
        p_cLoadEepromBinToSDRAM[2] = p_cDevicePartitionNumber;
        p_cLoadScriptBinToSDRAM[2] = p_cDevicePartitionNumber;

        // Update without DHupdate.ini file --> Command line update
        if(argc > 1) {
                cmd = argv[1];

                // StartUp automatic update with DHupdate.ini file
                if (strcmp(cmd, "auto") == 0) {
                        update_ini = UPDATE_INI_FILE;    // Set indicator for automatic update
                } else if (strcmp(cmd, "bootloader") == 0) {
                        cUpdateArgument = BOOTLOADER_FLASH_UPDATE;
                        p_vUpdateArgument = &cUpdateArgument;
                        update_ini = UPDATE_INI_CMDLINE;
                        // Load File name from command line
                        if(argc > 2) {
                                file_name = argv[2];
                                p_cLoadUBootBinToSDRAM[4] = file_name;
                        }
                } else if (strcmp(cmd, "eeprom") == 0) {
                        cUpdateArgument = EEPROM_SETTINGS_UPDATE;
                        p_vUpdateArgument = &cUpdateArgument;
                        update_ini = UPDATE_INI_CMDLINE;
                        // Load File name from command line
                        if(argc > 2) {
                                file_name = argv[2];
                                p_cLoadEepromBinToSDRAM[4] = file_name;
                        }
                } else if (strcmp(cmd, "script") == 0) {
                        cUpdateArgument = EXECUTE_BOOTLOADER_SCRIPT;
                        p_vUpdateArgument = &cUpdateArgument;
                        update_ini = UPDATE_INI_CMDLINE;
                        // Load File name from command line
                        if(argc > 2) {
                                file_name = argv[2];
                                p_cLoadScriptBinToSDRAM[4] = file_name;
                        }
                } else {
                        // Wrong argument
                        update_ini = UPDATE_INI_ERROR;

                        printf ("\n--> Update ERROR: Wrong command line update argument\n");
                        goto usage;
                }
        }

        // *************************************************************************************************************************************************
        // Handle DHupdate.ini File
        if(update_ini == UPDATE_INI_FILE) {
                // Try to load DHupdate.ini file to SDRAM.
                ret_value = do_load_wrapper(NULL, 0, 5, p_cLoadDHUpdateIniToSDRAM);

                // Check if DHupdate.ini was found on Storage Device
                if((ret_value != 0)) {
                        printf ("\n--> Update INFO: No DHupdate.ini file found on Storage Device\n");
                        return 1;
                }

                // Read DHupdate.ini file content
                if(ReadDHupdateFile(p_stDHupdateINI, simple_strtoul (cDHUpdateIniSDRAMAddress, NULL, 16))) {
                        printf("\n--> Update ERROR: Wrong DHupdate.ini file content \n");
                        return 1;
                }

                // Check if DHupdate.ini is a valid update file.
                if(memcmp ( (char*)p_stDHupdateINI->p_cValidMarker, p_cCompareString, sizeof(p_cCompareString))) {
                        printf ("\n--> Update ERROR: No valid DHupdate.ini file found on Storage Device\n");
                        // Don't Start OS after Update error
                        ////CLEAR_DH_GD_FLAG(BOOT_AFTER_UPDATE_FLAG);
                        return 1;
                }

                printf ("\n--> Update: DHupdate.ini (%d bytes) found on Storage Device\n", (int)simple_strtoul (getenv("filesize"), NULL, 16));
        }

        // *************************************************************************************************************************************************
        // Check if DHupdate.ini contains [display] section
        // If so, load progress bitmap.
        if(p_stDHupdateINI->iDisplayInfo == 1) {
                for(i = 0; i < p_stDHupdateINI->iUpdateCounter; i++) {
                        if (strcmp(p_stDHupdateINI->stDHUpdateInfo[i].p_cUpdateType, "refresh") == 0) {
                                iRefreshStringFound = 1;
                                break;
                        }
                }

                if(iRefreshStringFound == 0) {
                        if(ShowBitmap(p_stDHupdateINI, PROGRESS_BITMAP, p_cStorageDevice, p_cDevicePartitionNumber) != 0)
                                printf ("\n--> Update INFO: Progress bitmap %s not found", p_stDHupdateINI->p_cFileNameProgressBmp);
                }
        }

        // *************************************************************************************************************************************************
        // Check if DHupdate.ini contains [led] section
        // If so, init. update led
        if(p_stDHupdateINI->iLedInfo == 1) {
                if(DHCOMUpdateLED_Init(p_stDHupdateINI) != 0) {
                        printf ("\n--> Update INFO: Can't initialize update LED Pin %s", p_stDHupdateINI->p_cUpdateGpioName);
                        p_stDHupdateINI->iLedInfo = 0;
                } else {
                        // Activate Update GPIO
                        ActivateUpdateGPIO(p_stDHupdateINI);
                }
        }

        // *************************************************************************************************************************************************
        // ==> Start UPDATE LOOP
        do {
                // Identify the next specified update in DHupdate.ini file
                if(update_ini == UPDATE_INI_FILE)
                {
                        // Don't enable printf after update, if DIS_PRINTF in settings.bin file is set.
                        ////SET_DH_GD_FLAG(UPDATE_ACTIVE_FLAG);

                        if (strcmp(p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cUpdateType, "bootloader") == 0)
                        {
                                cUpdateArgument = BOOTLOADER_FLASH_UPDATE;
                                p_vUpdateArgument = &cUpdateArgument;

                                // Check if filename is specified in DHupdate.ini file
                                if(p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cFilename != NULL)
                                {
                                        p_cLoadUBootBinToSDRAM[4] = p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cFilename;
                                }
                        }
                        else if (strcmp(p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cUpdateType, "eeprom") == 0)
                        {
                                cUpdateArgument = EEPROM_SETTINGS_UPDATE;
                                p_vUpdateArgument = &cUpdateArgument;

                                // Check if filename is specified in DHupdate.ini file
                                if(p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cFilename != NULL)
                                {
                                        p_cLoadEepromBinToSDRAM[4] = p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cFilename;
                                }
                        }
                        else if (strcmp(p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cUpdateType, "script") == 0)
                        {
                                cUpdateArgument = EXECUTE_BOOTLOADER_SCRIPT;
                                p_vUpdateArgument = &cUpdateArgument;

                                // Check if filename is specified in DHupdate.ini file
                                if(p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cFilename != NULL)
                                {
                                        p_cLoadScriptBinToSDRAM[4] = p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cFilename;
                                }
                        }
                        else if (strcmp(p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cUpdateType, "refresh") == 0)
                        {
                                cUpdateArgument = REFRESH_DH_SETTINGS;
                                p_vUpdateArgument = &cUpdateArgument;
                        }
                        else if (strcmp(p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cUpdateType, "reset") == 0)
                        {
                                cUpdateArgument = EXECUTE_RESET;
                                p_vUpdateArgument = &cUpdateArgument;
                        }
                        else {
                                iLoadUpdateKernel = 1;
                                cUpdateArgument = LOAD_UPDATE_KERNEL_LATER;
                                p_vUpdateArgument = &cUpdateArgument;

                                /*if((strcmp(p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cUpdateType, "settings") == 0)) {
                                    // Set DH settings Filename for refresh command
                                    p_cLoadSettingsBinToSDRAM[4] = p_stDHupdateINI->stDHUpdateInfo[iUpdateLoopCounter].p_cFilename;
                                }*/
                        }
                }
                else
                {
                        // Run only one command line update
                        p_stDHupdateINI->iUpdateCounter = 1;
                }

                // Argument = '0' ==> Load Update Kernel later
                // Argument = '1' ==> Update Bootloader Flash Image
                // Argument = '2' ==> Refresh DH settings and LCD controller
                // Argument = '3' ==> Display adapter EEPROM settings update
                // Argument = '4' ==> Reset Board
                switch (*(char*)p_vUpdateArgument)
                {
                case LOAD_UPDATE_KERNEL_LATER:
                        // Do nothing!!! Load update kernel later...
                        break;

                // ***************************************************************************************
                // **************************** Bootloader Flash Image ***********************************
                // ***************************************************************************************
                case BOOTLOADER_FLASH_UPDATE:
                        printf ("\n==> Update: Start to Update Bootloader Flash Image");

                        // Load u-boot image file  from Storage Device to SDRAM.
                        printf("\n\nLoad u-boot with: %s, %s, %s, %s, %s \n", p_cLoadUBootBinToSDRAM[0], p_cLoadUBootBinToSDRAM[1], p_cLoadUBootBinToSDRAM[2], p_cLoadUBootBinToSDRAM[3], p_cLoadUBootBinToSDRAM[4]);

                        ret_value = do_load_wrapper(NULL, 0, 5, p_cLoadUBootBinToSDRAM);
                        if(ret_value == 0) {
                                printf ("\n--> Update: Load %s to SDRAM (%d bytes)", (char*)p_cLoadUBootBinToSDRAM[4], (int)simple_strtoul (getenv("filesize"), NULL, 16));

                                // Calculate necessary sectors in flash for the u-boot image file.
                                ulFilesize = simple_strtoul (getenv("filesize"), NULL, 16);
                                ulBlocks = (ulFilesize+ulBootloaderOffset) / ulFlashBlockSize + 0x1;

                                // Check if the file fits into to the flash-partition
                                if (check_imagesize (ulFilesize+ulBootloaderOffset, ulBootloaderFlashPartitionSize, BOOTLOADER_FLASH_UPDATE, cErrorString ))
                                {
                                        // ERROR: file is to large for the specified partition
                                        ShowUpdateError(p_stDHupdateINI, cErrorString, IMAGE_SIZE_ERROR, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                                        return 1;
                                }

                                printf ("\n--> Update: The new u-boot image File needs %lu Flash blocks\n", ulBlocks);
                                printf ("    e = Erase Block\n");
                                printf ("    w = Write to Block\n");
                                printf ("    d = Done\n");
                                printf ("    Write File to Flash:[");

                                ret_value = update_flash_content (ulBootloaderOffset, ulSDRAMBufferAddress, ulBlocks, ulFlashBlockSize);

                                if((ret_value & 0x3) == 1)
                                {
                                        sprintf (&cErrorString[0], "\n--> Update ERROR: Erase error on block 0x%08x\n", (unsigned int)(ulBootloaderOffset + (ret_value >> 2) * ulFlashBlockSize));
                                        ShowUpdateError(p_stDHupdateINI, &cErrorString[0], FLASH_ERROR, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                                        return 1;
                                }
                                else if((ret_value & 0x3) == 2)
                                {
                                        sprintf (&cErrorString[0], "\n--> Update ERROR: Write error on block 0x%08x\n", (unsigned int)(ulBootloaderOffset + (ret_value >> 2) * ulFlashBlockSize));
                                        ShowUpdateError(p_stDHupdateINI, &cErrorString[0], FLASH_ERROR, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                                        return 1;
                                }

                                // Bootloader Flash Update done.
                                printf ("\n--> Update: U-Boot Update done\n");
                        }
                        else
                        {
                                // Can't load u-boot image file from Storage Device to SDRAM.
                                sprintf (&cErrorString[0], "\n--> Update ERROR: No %s file found on Storage Device\n", (char*)p_cLoadUBootBinToSDRAM[4]);
                                ShowUpdateError(p_stDHupdateINI, &cErrorString[0], FILE_NOT_FOUND_ERROR, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                                return 1;
                        }
                        break;
                // ***************************************************************************************
                // **************************** Update eeprom.bin File *********************************
                // ***************************************************************************************
                case EEPROM_SETTINGS_UPDATE:
                        printf ("\n==> Update: Start to Update display adapter eeprom content\n");
                        // Load eeprom file from Storage Device to SDRAM.
                        ret_value = do_load_wrapper(NULL, 0, 5, p_cLoadEepromBinToSDRAM);
                        if(ret_value == 0) {
                                printf ("\n--> Update: Load %s to SDRAM (%d bytes)", (char*)p_cLoadEepromBinToSDRAM[4], (int)simple_strtoul (getenv("filesize"), NULL, 16));

                                // Get eeprom.bin filesize
                                ulFilesize = simple_strtoul (getenv("filesize"), NULL, 16);

                                // Check if the file fits into to the eeprom-partition
                                if (check_imagesize (ulFilesize, DHCOM_DISPLAY_SETTINGS_SIZE, EEPROM_SETTINGS_UPDATE, cErrorString ))
                                {
                                        // ERROR: file is to large for the specified partition
                                        ShowUpdateError(p_stDHupdateINI, cErrorString, IMAGE_SIZE_ERROR, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                                        return 1;
                                }

                                printf ("\n--> Update: The new eeprom File needs %lu Bytes\n", ulFilesize);
                                printf ("    w = Write to addr\n");
                                printf ("    d = Done\n");
                                printf ("    Write File to EEPROM:[");


                                ret_value = update_eeprom_content (0x00, ulSDRAMBufferAddress, ulFilesize);

                                if(ret_value != 0)
                                {
                                        sprintf (&cErrorString[0], "\n--> Update ERROR: EEPROM write error\n");
                                        ShowUpdateError(p_stDHupdateINI, &cErrorString[0], FLASH_ERROR, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                                        return 1;
                                }

                                // Settings EEPROM Update done.
                                printf ("\n--> Update: Settings Update done\n");
                        }
                        else
                        {
                                // Can't load settings file from Storage Device to SDRAM.
                                sprintf (&cErrorString[0], "\n--> Update ERROR: No %s file found on Storage Device\n", (char*)p_cLoadEepromBinToSDRAM[4]);
                                ShowUpdateError(p_stDHupdateINI, &cErrorString[0], FILE_NOT_FOUND_ERROR, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                                return 1;
                        }
                        break;
                // ***************************************************************************************
                // ****************************** Refresh DH settings ***********************************
                // ***************************************************************************************
                case REFRESH_DH_SETTINGS:
                        printf ("\n==> Update: Refresh NOT IMPLEMENTED AT THE MOMENT!!!");

                        if(ShowBitmap(p_stDHupdateINI, PROGRESS_BITMAP, p_cStorageDevice, p_cDevicePartitionNumber) != 0)
                        {
                                printf ("\n--> Update INFO: Progress bitmap %s not found", p_stDHupdateINI->p_cFileNameProgressBmp);
                        }
                        /*printf ("\n==> Update: Refresh DH settings");

                        if(UpdateGlobalDataDHSettings(ulSDRAMBufferAddress) == 0)
                        {
                            //lcd_ctrl_init ((void *)(gd->fb_base));

                            // Check if DHupdate.ini contains [display] section
                            // If so, load progress bitmap.
                            if(ShowBitmap(p_stDHupdateINI, PROGRESS_BITMAP, p_cStorageDevice, p_cDevicePartitionNumber) != 0) {
                                printf ("\n--> Update INFO: Progress bitmap %s not found", p_stDHupdateINI->p_cFileNameProgressBmp);
                            }
                            //board_video_skip();

                            //video_init();
                            //drv_video_init();

                            // Refresh DH settings done.
                            printf ("\n--> Update: Refresh DH settings done\n");
                        }
                        else
                        {
                            // No valid DH settings FIle found on SDRAM
                            sprintf (&cErrorString[0], "\n--> Update ERROR: No valid DH settings file found on RAM (Note: Set \"refresh\" string after \"settings\" update)\n");
                            ShowUpdateError(p_stDHupdateINI, &cErrorString[0], FILE_NOT_FOUND_ERROR, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                            return 1;
                        }*/
                        printf ("\n");
                        break;
                // ***************************************************************************************
                // *************************** Execute Bootloader Script *********************************
                // ***************************************************************************************
                case EXECUTE_BOOTLOADER_SCRIPT:
                        printf ("\n==> Update: Execute Bootloader Script");

                        ret_value = do_load_wrapper(NULL, 0, 5, p_cLoadScriptBinToSDRAM);
                        if(ret_value == 0) {
                                printf ("\n--> Update: Load %s to SDRAM (%d bytes)", (char*)p_cLoadScriptBinToSDRAM[4], (int)simple_strtoul (getenv("filesize"), NULL, 16));

                                printf ("\n\n----------------- Print Script Info: -----------------\n");
                                do_source(NULL, 0, 2, p_cRunScript);
                                printf ("\n----------------------------------------------------------\n");
                        } else {
                                // Can't load script file from Storage Device to SDRAM.
                                sprintf (&cErrorString[0], "\n--> Update ERROR: No %s file found on Storage Device\n", (char*)p_cLoadScriptBinToSDRAM[4]);
                                ShowUpdateError(p_stDHupdateINI, &cErrorString[0], FILE_NOT_FOUND_ERROR, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                                return 1;
                        }

                        // Script Execution done.
                        printf ("\n--> Update: Script Execution done\n");
                        break;
                // ***************************************************************************************
                // ****************************** Reset Board ********************************************
                // ***************************************************************************************
                case EXECUTE_RESET:

                        if(p_stDHupdateINI->iUpdateCounter != 1)
                        {
                                printf ("\n==> Update ERROR: Skipped reset!!!");

                                // Can't do reset.
                                sprintf (cErrorString, "\n==> Update ERROR: Reset has to be the last action in DHupdate.ini (Danger of infinite Update-Loop)!!!");
                                ShowUpdateError(p_stDHupdateINI, cErrorString, DHUPDATE_INI_ERROR, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                                return 1;
                        }
                        if(iLoadUpdateKernel == 0)
                        {
                                printf ("\n==> Update: Reset Board");
                                // do reset
                                run_command("reset",0);
                        }

                        break;
                // ***************************************************************************************
                // *** No Update Event is defined for the Update Argument in the DHupdate.ini file ***
                // ***************************************************************************************
                default:
                        if(update_ini == UPDATE_INI_FILE)
                        {
                                sprintf (&cErrorString[0], "\n--> Update ERROR: No Update Event defined for the Argument: %s\n", (char*)p_vUpdateArgument);
                                ShowUpdateError(p_stDHupdateINI, &cErrorString[0], DHUPDATE_INI_ERROR, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                                return 1;
                        }
                        else
                        {
                                printf ("\n--> Update ERROR: No Update Event defined for this command line Argument\n");
                                // Don't Start OS after Update error
                                goto usage;
                        }
                }

                if(update_ini == UPDATE_INI_FILE)
                {
                        // Start next update
                        iUpdateLoopCounter++;
                }

                // Number of updates
                p_stDHupdateINI->iUpdateCounter--;

        } while(p_stDHupdateINI->iUpdateCounter > 0);
        // ==> End UPDATE LOOP
        // *************************************************************************************************************************************************

        // ==> Start Update Kernel
        if(iLoadUpdateKernel == 1) {
                if ((cmd = getenv ("load_update_kernel")) == NULL) {
                        sprintf (&cErrorString[0], "\n--> Update ERROR: \"load_update_kernel\" not defined\n");
                        ShowUpdateError(p_stDHupdateINI, &cErrorString[0], CANT_LOAD_UPDATE_KERNEL, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                        return 1;
                }

                printf ("\n==> Update: Start to Load Update Kernel\n");

                // Set current Update device and partition for update kernel
                setenv("src_intf", p_cStorageDevice);
                setenv("src_dev_part", p_cDevicePartitionNumber);

                run_command (cmd, 0);
                // If run command returns --> show error
                sprintf (&cErrorString[0], "\n--> Update ERROR: Command \"load_update_kernel\"\n");
                ShowUpdateError(p_stDHupdateINI, &cErrorString[0], CANT_LOAD_UPDATE_KERNEL, update_ini, p_cStorageDevice, p_cDevicePartitionNumber);
                return 1;
        }

        if(ShowBitmap(p_stDHupdateINI, END_BITMAP, p_cStorageDevice, p_cDevicePartitionNumber) != 0) {
                printf ("\n--> Update INFO: End bitmap %s not found\n", p_stDHupdateINI->p_cFileNameOkBmp);
        }

        printf ("\n");

        return 0;

usage:
        cmd_usage(cmdtp);
        return 2;
}

//------------------------------------------------------------------------------
//
//  Function:  do_DHCOMupdate
//
//  DHCOM U-Boot Update function
//  Description:    Scan for available storage devices
//
//  Return value:   0 = No error
//                  1 = error
//
int do_DHCOMupdate(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
        char *p_cUSBInit[2]                     = {"usb","start"};
        char *p_cMMCDevSDCard[3]                = {"mmc","dev","0"};
        char *p_cMMCReScan[2]                   = {"mmc","rescan"};
        char p_cCompareString[]               = "##DHCOMupdate##";

        char cSDCardStorageDevice[] = "mmc";
        char cSDCardDevicePartitionNumber[] = "0:1";
        char cUSBStorageDevice[] = "usb";
        char cUSBHost1DevicePartitionNumber[] = "0:1";
        char cDHupdateIniInvalid[] = "INVALID";

        int ret_value = 0;
        int iUpdateDeviceCounter = 0;
        int iUpdateSuccess = 0;
        int iDHupdateIniFound = 0;
        int update_auto = 0;

        updateinfo_t stDHupdateINI;
        memset(&stDHupdateINI,0, sizeof(stDHupdateINI));

        if(argc > 1) {
                // Check for "update auto"
                if (strcmp(argv[1], "auto") == 0) {
                        update_auto = 1;
                }
        }

        // Check for available storage devices depending on the flags of the settings block
        for(iUpdateDeviceCounter = 1; (iUpdateDeviceCounter <= 4) && (iUpdateSuccess == 0) && (iDHupdateIniFound == 0); iUpdateDeviceCounter++)
        {
                stDHupdateINI.p_cValidMarker = cDHupdateIniInvalid;

                switch(iUpdateDeviceCounter)
                {
                case 1:
                        /* do nothing, we have only one SD Slot for MicroSD and SD */
                        break;
                case 2: // SD/MMC Card Slot
                        if(update_auto == 1) { /* check update media flags */
                                if ( !(gd->dh_board_settings.wHWConfigFlags & UPDATE_VIA_SD_MMC_SLOT ||
                                                gd->dh_board_settings.wHWConfigFlags & UPDATE_VIA_MICROSD_SLOT) ) {
                                        /* am335x special case: skip SD/µSD update if none of both flags is set */
                                        break;
                                }
                        }

                        // Call update function, if MMC/SD init returns success
                        do_mmcops(NULL, 0, 3, p_cMMCDevSDCard);
                        ret_value = do_mmcops(NULL, 0, 2, p_cMMCReScan);
                        if(ret_value == 0) {
                                ret_value = DHCOMupdate (cmdtp, argc, argv, &stDHupdateINI, &cSDCardStorageDevice[0], &cSDCardDevicePartitionNumber[0]);

                                if(ret_value == 0) { // Update success
                                        // Don't try to start update from another storage device
                                        iUpdateSuccess = 1;
                                } else if(ret_value == 2) { // Wrong update argument. Usage was called...
                                        return 1;
                                } else if(!(memcmp ( (char*)stDHupdateINI.p_cValidMarker, p_cCompareString, sizeof(p_cCompareString) )) ) { // DHupdate.ini File found on storage device, but Update fails with error
                                        iDHupdateIniFound = 1; // Don't try to start update from another storage device
                                } else { // Command line Update with error --> New line is neccessary
                                        printf ("\n");
                                }
                        } else {
                                printf ("\n--> Update INFO: No MMC/SD - Card detected!\n");
                        }
                        break;
                case 3: // USB Host 1 Port
                        if(update_auto == 1) { /* check update media flags */
                                if ( !(gd->dh_board_settings.wHWConfigFlags & UPDATE_VIA_USB_HOST_1_PORT) ) {
                                        /* skip USB Host 1 update */
                                        break;
                                }
                        }

                        // Initialize USB Stick
                        printf ("--> Update: Try to initialize USB Stick on Host Port:");
                        ret_value = do_usb(NULL, 0, 2, p_cUSBInit);
                        if(ret_value == 0) { // Call update function, if USB init returns success
                                ret_value = DHCOMupdate (cmdtp, argc, argv, &stDHupdateINI, &cUSBStorageDevice[0], &cUSBHost1DevicePartitionNumber[0]);
                                if(ret_value == 0) {
                                        // Don't try to start update from another storage device
                                        iUpdateSuccess = 1;
                                } else if(ret_value == 2) { // Wrong update argument. Usage was called...
                                        return 1;
                                } else if( 0 == (memcmp ( (char*)stDHupdateINI.p_cValidMarker, p_cCompareString, sizeof(p_cCompareString) )) ) { // DHupdate.ini File found on storage device, but Update fails with error
                                        iDHupdateIniFound = 1; // Don't try to start update from another storage device
                                } else { // Command line Update with error --> New line is neccessary
                                        printf ("\n");
                                }
                        } else {
                                printf ("\n--> Update INFO: No USB Stick detected!\n");
                        }
                        break;
                case 4: // USB OTG Port
                        if(update_auto == 1) { /* check update media flags */
                                if ( !(gd->dh_board_settings.wHWConfigFlags & UPDATE_VIA_USB_OTG_PORT) ) {
                                        /* skip USB Host 1 update */
                                        break;
                                }
                        }
                        printf ("\n--> Update INFO: USB OTG update is not supported yet!\n");
                        break;
                default:
                        printf ("\n--> Update INFO: Unsupported update media!\n");
                        break;
                }
        }

        return 0;
}

/* ====================================================================== */

U_BOOT_CMD(
        update,      3,      0,      do_DHCOMupdate,
        "Update Flash content from Storage Device.",
        "- Starts update with DHupdate.ini file\n"
        "         Necessary files on Storage Device:\n"
        "           - DHupdate.ini File\n"
        "           - uboot.imx, ...\n"
        "update <type> [filename] - Starts update without DHupdate.ini file\n"
        "         Types:\n"
        "           - bootloader = Bootloader update (default file name u-boot.imx)\n"
        "           - eeprom = Display adpater EEPROM update (default file name eeprom.bin)\n"
        "           - script = Run bootloader script (default file name script.bin)\n"
        "           - auto = Run DHupdate.ini update from command line\n"
);
#endif/* CONFIG_CMD_UPDATECE */
