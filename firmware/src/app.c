/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

/* This is the string that will written to the file */
uint8_t __attribute__((coherent)) writeData[12] = "Hello World";
uint8_t __attribute__((coherent)) readData[16];

const SYS_FS_MEDIA_FUNCTIONS sst25MediaFunctions =
{
    .mediaStatusGet     = DRV_SST25_MediaIsAttached,
    .mediaGeometryGet   = DRV_SST25_GeometryGet,
    .sectorRead         = DRV_SST25_BlockRead,
    .sectorWrite        = DRV_SST25_BlockEraseWrite,
    .eventHandlerset    = DRV_SST25_BlockEventHandlerSet,
    .commandStatusGet   = (void *)DRV_SST25_CommandStatus,
    .Read               = DRV_SST25_BlockRead,
    .erase              = DRV_SST25_BlockErase,
    .addressGet         = DRV_SST25_AddressGet,
    .open               = DRV_SST25_Open,
    .close              = DRV_SST25_Close,
    .tasks              = DRV_SST25_Tasks,
};

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.data = &readData[0];
};


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
            if (appInitialized) {
                SYS_CONSOLE_MESSAGE("System initialization finished.\r\n");
            }
#if 1
            appData.state = APP_REGISTER;
            SYS_CONSOLE_MESSAGE("Next step: register the file system.\r\n");
#else
            appData.state = APP_MOUNT_DISK;
            SYS_CONSOLE_MESSAGE("Next step: mount the disk.\r\n");
#endif
            break;
        }

        case APP_REGISTER:
            {
                if (DRV_SST25_Status(sysObj.drvSst25Obj0) == SYS_STATUS_READY) {
                     SYS_CONSOLE_MESSAGE("Register SST25 with SysFs.\r\n");
                     if (SYS_FS_MEDIA_MANAGER_Register((SYS_MODULE_OBJ)DRV_SST25_INDEX_0,
                                                       (SYS_MODULE_INDEX)DRV_SST25_INDEX_0,
                                                       &sst25MediaFunctions,
                                                       SYS_FS_MEDIA_TYPE_SPIFLASH) != SYS_FS_MEDIA_HANDLE_INVALID) {
                       SYS_CONSOLE_MESSAGE("    => SUCCESS!\r\n");
                       SYS_CONSOLE_MESSAGE("Next step: mount the disk.\r\n");
                       appData.state = APP_MOUNT_DISK;
                     } else {
                         SYS_CONSOLE_MESSAGE("    => FAIL!\r\n");
                         appData.state = APP_ERROR;
                     }
                }
                break;
            }

        case APP_MOUNT_DISK:
            {
                /* Mount the disk */
                if (SYS_FS_Mount("/dev/mtda1", "/mnt/myDrive", FAT, 0, NULL) != 0) {
                    /* The disk could not be mounted. Try mounting again until
                     * the operation succeeds. */
                    appData.state = APP_MOUNT_DISK;
                } else {
                    /* Mount was successful. Format the disk. */
                    SYS_CONSOLE_MESSAGE("Disk mounted, next step: format.\r\n");
                    appData.state = APP_FORMAT_DISK;
                }
                break;
            }

        case APP_FORMAT_DISK:
            {
                SYS_CONSOLE_MESSAGE("Formatting the drive...\r\n");
                if (SYS_FS_DriveFormat ("/mnt/myDrive", SYS_FS_FORMAT_SFD, 0) != 0) {
                    /* Format of the disk failed. */
                    SYS_CONSOLE_MESSAGE("Error formatting the drive.\r\n");
                    appData.state = APP_ERROR;
                } else {
                    /* Format succeeded. Open a file. */
                    appData.state = APP_OPEN_FILE;
                    SYS_CONSOLE_MESSAGE("Drive is formatted, next step: open file.\r\n");
                }
                break;
            }

        case APP_OPEN_FILE:
            {
                SYS_CONSOLE_MESSAGE("Opening the file...\r\n");
                appData.fileHandle = SYS_FS_FileOpen("newFile.txt", (SYS_FS_FILE_OPEN_APPEND_PLUS));
                if (appData.fileHandle == SYS_FS_HANDLE_INVALID) {
                    /* File open unsuccessful */
                    appData.state = APP_ERROR;
                    SYS_CONSOLE_MESSAGE("Failed to open the file.\r\n");
                } else {
                    /* File open was successful. Write to the file. */
                    appData.state = APP_WRITE_TO_FILE;
                    SYS_CONSOLE_MESSAGE("File is opened, begin write sequence.\r\n");
                }
                break;
            }

        case APP_WRITE_TO_FILE:
            {
                SYS_CONSOLE_MESSAGE("Writing the file...\r\n");
                SYS_CONSOLE_PRINT("Data to write: %s\r\n", writeData);
                if (SYS_FS_FileWrite (appData.fileHandle, (void *)writeData, 12) == -1) {
                    /* Failed to write to the file. */
                    appData.state = APP_ERROR;
                    SYS_CONSOLE_MESSAGE("Error writing to file.\r\n");
                } else {
                    /* File write was successful. */
                    appData.state = APP_FLUSH_FILE;
                    SYS_CONSOLE_MESSAGE("File is written, next step is flush.\r\n");
                }
                break;
            }

        case APP_FLUSH_FILE:
            {
                SYS_CONSOLE_MESSAGE("Flushing the file...\r\n");
                if (SYS_FS_FileSync(appData.fileHandle) != 0) {
                    /* Could not flush the contents of the file. Error out. */
                    appData.state = APP_ERROR;
                    SYS_CONSOLE_MESSAGE("Error flushing the file.\r\n");
                } else {
                    /* Check the file status */
                    appData.state = APP_READ_FILE_STAT;
                    SYS_CONSOLE_MESSAGE("File is flushed, next step: stat the file.\r\n");
                }
                break;
            }

        case APP_READ_FILE_STAT:
            {
                SYS_CONSOLE_MESSAGE("Stating the file...\r\n");
                if (SYS_FS_FileStat("/mnt/myDrive/newFile.txt", &appData.fileStatus) == SYS_FS_RES_FAILURE) {
                    /* Reading file status was a failure */
                    appData.state = APP_ERROR;
                    SYS_CONSOLE_MESSAGE("Error stating the file.\r\n");
                } else {
                    /* Read file size */
                    appData.state = APP_READ_FILE_SIZE;
                    SYS_CONSOLE_MESSAGE("File is stated, next step: read file size.\r\n");
                    SYS_CONSOLE_PRINT("> File name: %s\r\n", appData.fileStatus.fname);
                    SYS_CONSOLE_PRINT("> File size: %d\r\n", appData.fileStatus.fsize);
                }
                break;
            }

        case APP_READ_FILE_SIZE:
            {
                SYS_CONSOLE_MESSAGE("Reading file size...\r\n");
                appData.fileSize = SYS_FS_FileSize(appData.fileHandle);
                if (appData.fileSize == -1) {
                    /* Reading file size was a failure */
                    SYS_CONSOLE_MESSAGE("Error reading file size.\r\n");
                    appData.state = APP_ERROR;
                } else {
                    if (appData.fileSize == appData.fileStatus.fsize) {
                        appData.state = APP_DO_FILE_SEEK;
                        SYS_CONSOLE_PRINT("> File size: %d\r\n", appData.fileSize);
                        SYS_CONSOLE_MESSAGE("File size matched, next step: file seek.\r\n");
                    } else {
                        appData.state = APP_ERROR;
                        SYS_CONSOLE_MESSAGE("File is an unexpected size.\r\n");
                    }
                }
                break;
            }

        case APP_DO_FILE_SEEK:
            {
                SYS_CONSOLE_MESSAGE("Doing file seek...\r\n");
                if (SYS_FS_FileSeek(appData.fileHandle, appData.fileSize, SYS_FS_SEEK_SET) == -1) {
                    /* File seek caused an error */
                    appData.state = APP_ERROR;
                    SYS_CONSOLE_MESSAGE("Error during seek.\r\n");
                } else {
                    /* Check for End of file */
                    appData.state = APP_CHECK_EOF;
                    SYS_CONSOLE_MESSAGE("Seek is done, next step: check EOF.\r\n");
                }
                break;
            }

        case APP_CHECK_EOF:
            {
                SYS_CONSOLE_MESSAGE("Checking EOF...\r\n");
                if (SYS_FS_FileEOF(appData.fileHandle) == false ) {
                    /* Either, EOF is not reached or there was an error
                       In any case, for the application, its an error condition
                       */
                    appData.state = APP_ERROR;
                    SYS_CONSOLE_MESSAGE("Error during EOF check.\r\n");
                } else {
                    appData.state = APP_DO_ANOTHER_FILE_SEEK;
                    SYS_CONSOLE_MESSAGE("EOF checked, next step: do another seek.\r\n");
                }
                break;
            }

        case APP_DO_ANOTHER_FILE_SEEK:
            {
                SYS_CONSOLE_MESSAGE("Moving pointer to beginning...\r\n");
                /* Move file pointer to beginning of file */
                if (SYS_FS_FileSeek(appData.fileHandle, 0, SYS_FS_SEEK_SET) == -1) {
                    /* File seek caused an error */
                    appData.state = APP_ERROR;
                    SYS_CONSOLE_MESSAGE("Error during file seek.\r\n");
                } else {
                    /* Check for original file content */
                    appData.state = APP_READ_FILE_CONTENT;
                    SYS_CONSOLE_MESSAGE("File seek reset, next step: read file content.\r\n");
                }
                break;
            }

        case APP_READ_FILE_CONTENT:
            {
                SYS_CONSOLE_MESSAGE("Reading file content...\r\n");
                if (SYS_FS_FileRead(appData.fileHandle, (void *)appData.data, appData.fileSize) == -1) {
                    /* There was an error while reading the file. Close the file
                     * and error out. */
                    SYS_FS_FileClose(appData.fileHandle);
                    appData.state = APP_ERROR;
                    SYS_CONSOLE_MESSAGE("Error during read.\r\n");
                } else {
                    if ((appData.fileSize != 12) || (memcmp(appData.data, writeData, 12) != 0)) {
                        /* The written and the read data don't match. */
                        appData.state = APP_ERROR;
                        SYS_CONSOLE_MESSAGE("Unexpected number of bytes read.\r\n");
                    } else {
                        /* The test was successful. */
                        appData.state = APP_CLOSE_FILE;
                        SYS_CONSOLE_MESSAGE("Successful read, next step: close the file.\r\n");
                        SYS_CONSOLE_PRINT("> Read buffer: %s\r\n", appData.data);
                    }
                }
                break;
            }

        case APP_CLOSE_FILE:
            {
                /* Close the file */
                SYS_CONSOLE_MESSAGE("Closing the file....\r\n");
                if (SYS_FS_FileClose(appData.fileHandle) != 0) {
                    appData.state = APP_ERROR;
                    SYS_CONSOLE_MESSAGE("Error while closing the file.\r\n");
                } else {
                    appData.state = APP_IDLE;
                    SYS_CONSOLE_MESSAGE("All tests finished!\r\n");
                }
                break;
            }

        case APP_IDLE:
            {
                break;
            }

        case APP_ERROR:
            {
                /* The application comes here when the demo has failed. */
                SYS_CONSOLE_MESSAGE("Error happened.\r\n");
                appData.state = APP_IDLE;
                break;
            }
    }
}

/*******************************************************************************
 End of File
 */
