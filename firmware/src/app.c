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
            appData.state = APP_STATE_SERVICE_TASKS;
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            static bool fs_registered = false;
            static bool fs_mounted = false;
            static bool fs_tested = false;
            if (!fs_registered &&
                 DRV_SST25_Status(sysObj.drvSst25Obj0) == SYS_STATUS_READY) {
                 SYS_CONSOLE_MESSAGE("Register SST25 with SysFs.\r\n");
                 if (SYS_FS_MEDIA_MANAGER_Register((SYS_MODULE_OBJ)DRV_SST25_INDEX_0,
                                                   (SYS_MODULE_INDEX)DRV_SST25_INDEX_0,
                                                   &sst25MediaFunctions,
                                                   SYS_FS_MEDIA_TYPE_SPIFLASH) != SYS_FS_MEDIA_HANDLE_INVALID) {
                   SYS_CONSOLE_MESSAGE("    => SUCCESS!\r\n");
                 }
                 fs_registered = true;
                 break;
            }
            if (fs_registered && !fs_mounted) {
                if (SYS_FS_Mount("/dev/mtda1", "/mnt/myDrive", FAT, 0, NULL) != 0) {
                    /* The disk could not be mounted. Try mounting again until
                     * mount is successful. */
                } else {
                    SYS_CONSOLE_MESSAGE("MOUNTED.\r\n");
                    fs_mounted = true;
                    break;
                }
            }
            if (fs_mounted && !fs_tested) {
                uint32_t totalSectors, freeSectors;
                SYS_FS_RESULT res;
                res = SYS_FS_DriveSectorGet("/mnt/myDrive", &totalSectors, &freeSectors);
                if (res == SYS_FS_RES_SUCCESS) {
                    SYS_CONSOLE_PRINT("Total sectors : %d.\r\n", totalSectors);
                    SYS_CONSOLE_PRINT("Free sectors  : %d.\r\n", freeSectors);
                } else {
                  SYS_CONSOLE_MESSAGE("Error in SYS_FS_DriveSectorGet.\r\n");
                }
                fs_tested = true;
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

/*******************************************************************************
 End of File
 */
