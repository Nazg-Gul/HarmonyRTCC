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
            // Initialize RTC handles.
            if (RTC_MCP7940N_Initialize(&appData.rtc, DRV_I2C_INDEX_0)) {
              SYS_CONSOLE_MESSAGE("APP: RTC successfully initialized.\r\n");
            } else  {
              appInitialized = false;
            }
            // Check overall status.
            if (appInitialized) {
                SYS_CONSOLE_MESSAGE("APP: Initialization successful.\r\n");
                appData.state = APP_STATE_TEST_WRITE_DATE_TIME;
                // appData.state = APP_STATE_TEST_READ_DATE_TIME;
            } else {
                SYS_CONSOLE_MESSAGE("APP: Initialization failed!\r\n");
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_TEST_WRITE_DATE_TIME:
        {
            SYS_CONSOLE_MESSAGE("APP: Setting RTC date and time.\r\n");
            RTC_MCP7940N_DateTime date_time;
            date_time.seconds = 13;
            date_time.minutes = 38;
            date_time.hours = 20;
            date_time.day = 4;
            date_time.month = 6;
            date_time.year = 17;
            RTC_MCP7940N_WriteDateAndTime(&appData.rtc, &date_time);
            appData.state = APP_STATE_TEST_WAIT_DATE_TIME;
            break;
        }
        case APP_STATE_TEST_WAIT_DATE_TIME:
        {
            if (RTC_MCP7940N_IsBusy(&appData.rtc)) {
                RTC_MCP7940N_Tasks(&appData.rtc);
            } else {
                SYS_CONSOLE_MESSAGE("APP: Date and time is set.\r\n");
                appData.state = APP_STATE_TEST_ENABLE_OSCILLATOR;
            }
            break;
        }

        case APP_STATE_TEST_READ_REGISTER:
        {
            RTC_MCP7940N_ReadRegister(&appData.rtc,
                                      MCP7940N_REG_ADDR_SECONDS,
                                      &appData.register_value);
            appData.state = APP_STATE_TEST_WAIT_READ_REGISTER;
            break;
        }
        case APP_STATE_TEST_WAIT_READ_REGISTER:
        {
            if (RTC_MCP7940N_IsBusy(&appData.rtc)) {
                RTC_MCP7940N_Tasks(&appData.rtc);
            } else {
                SYS_CONSOLE_MESSAGE("APP: Finished reading register.\r\n");
                SYS_CONSOLE_PRINT("APP: Register value: %d\r\n",
                                  appData.register_value);
                appData.state = APP_STATE_TEST_READ_DATE_TIME;
            }
            break;
        }

        case APP_STATE_TEST_ENABLE_OSCILLATOR:
        {
            RTC_MCP7940N_EnableOscillator(&appData.rtc, true);
            appData.state = APP_STATE_TEST_WAIT_ENABLE_OSCILLATOR;
            break;
        }

        case APP_STATE_TEST_WAIT_ENABLE_OSCILLATOR:
        {
            if (RTC_MCP7940N_IsBusy(&appData.rtc)) {
                RTC_MCP7940N_Tasks(&appData.rtc);
            } else {
                SYS_CONSOLE_MESSAGE("APP: Oscillator STARTED!\r\n");
                appData.state = APP_STATE_TEST_ENABLE_BATTERY;
            }
            break;
        }

        case APP_STATE_TEST_ENABLE_BATTERY:
        {
            RTC_MCP7940N_EnableBatteryBackup(&appData.rtc, true);
            appData.state = APP_STATE_TEST_WAIT_ENABLE_BATTERY;
            break;
        }

        case APP_STATE_TEST_WAIT_ENABLE_BATTERY:
        {
            if (RTC_MCP7940N_IsBusy(&appData.rtc)) {
                RTC_MCP7940N_Tasks(&appData.rtc);
            } else {
                SYS_CONSOLE_MESSAGE("APP: Battery ENABLED!\r\n");
                appData.state = APP_STATE_TEST_READ_REGISTER;
            }
            break;
        }

        case APP_STATE_TEST_READ_DATE_TIME:
        {
            RTC_MCP7940N_ReadDateAndTime(&appData.rtc, &appData.date_time);
            appData.state = APP_STATE_TEST_WAIT_READ_DATE_TIME;
            break;
        }

        case APP_STATE_TEST_WAIT_READ_DATE_TIME:
        {
            if (RTC_MCP7940N_IsBusy(&appData.rtc)) {
                RTC_MCP7940N_Tasks(&appData.rtc);
            } else {
                SYS_CONSOLE_MESSAGE("APP: Date was read.\r\n");
                SYS_CONSOLE_PRINT("  Seconds: %d\r\n", appData.date_time.seconds);
                SYS_CONSOLE_PRINT("  Minutes: %d\r\n", appData.date_time.minutes);
                SYS_CONSOLE_PRINT("  Hours: %d\r\n", appData.date_time.hours);
                SYS_CONSOLE_PRINT("  Day of week: %d\r\n", appData.date_time.day_of_week);
                SYS_CONSOLE_PRINT("  Day: %d\r\n", appData.date_time.day);
                SYS_CONSOLE_PRINT("  Month: %d\r\n", appData.date_time.month);
                SYS_CONSOLE_PRINT("  Year: %d\r\n", appData.date_time.year);
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            if (RTC_MCP7940N_IsBusy(&appData.rtc)) {
                RTC_MCP7940N_Tasks(&appData.rtc);
            } else {
#if 1
                static int counter = 0;
                if (counter == 327680/* 256*/) {
                    appData.state = APP_STATE_TEST_READ_DATE_TIME;
                    counter = 0;
                } else {
                  counter++;
                }
#endif
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
