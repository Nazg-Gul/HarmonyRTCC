// Copyright (c) 2017, Sergey Sharybin
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//
// Author: Sergey Sharybin (sergey.vfx@gmail.com)

#ifndef _RTC_MCP7940N_H
#define _RTC_MCP7940N_H

#include <stdbool.h>
#include <stdint.h>

#include "system/common/sys_module.h"  // SYS function prototypes.

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
// I2C bus address of the RTC itself.

#define MCP7940N_I2C_ADDRESS    0x6f

////////////////////////////////////////////////////////////////////////////////
// Addresses of various registers.

#define MCP7940N_REG_ADDR_SECONDS      0x00
#define MCP7940N_REG_ADDR_MINUNTES     0x01
#define MCP7940N_REG_ADDR_HOURS        0x02
#define MCP7940N_REG_ADDR_DAY_OF_WEEK  0x03
#define MCP7940N_REG_ADDR_STATUS       0x03
#define MCP7940N_REG_ADDR_DATE         0x04
#define MCP7940N_REG_ADDR_MONTH        0x05
#define MCP7940N_REG_ADDR_YEAR         0x06
#define MCP7940N_REG_ADDR_CONTROL      0x07
#define MCP7940N_REG_ADDR_CALIB        0x08
#define MCP7940N_REG_ADDR_UNLOCK_ID    0x09

/////////////////////////////////////////////////////////////////////////////////
// Various flags.

////////////////////////////////////////////////
// Flags for MCP7940N_REG_ADDR_SECONDS register.

#define MCP7940N_FLAG_START_OSCILLATOR  0x80  /* Start crystal oscillator */

//////////////////////////////////////////////
// Flags for MCP7940N_REG_ADDR_MONTH register.

#define MCP7940N_FLAG_LEAP              0x20  /* mask for the leap year bit. */

/////////////////////////////////////////////
// Flags for MCP7940N_REG_ADDR_HOUR register.

#define MCP7940N_FLAG_HOUR_12           0x40  /* 12 hours format */
#define MCP7940N_FLAG_PM                0x20  /* Post-meridian bit */

////////////////////////////////////////////////////
// Flags for MCP7940N_REG_ADDR_DAY_OF_WEEK register.

#define MCP7940N_FLAG_BATTERY_ENABLE  0x08  /* Enable battery for back-up */

////////////////////////////////////////////////
// Flags for MCP7940N_REG_ADDR_CONTROL register.

#define MCP7940N_FLAG_OUT_PIN      0x80
#define MCP7940N_FLAG_SQWE         0x40  /* Square wave clock output mode. */
#define MCP7940N_FLAG_ALM_NONE     0x00  /* No alarms are activated. */
#define MCP7940N_FLAG_ALARM_0      0x10  /* ALARM0 is activated. */
#define MCP7940N_FLAG_ALARM_1      0x20  /* ALARM1 is activated. */
#define MCP7940N_FLAG_ALARM_ALL    (MCP7940N_FLAG_ALARM_0 | MCP7940N_FLAG_ALARM_1)
#define MCP7940N_FLAG_MFP_1HZ      0x00  /*  MFP is running at 1 Hz */
#define MCP7940N_FLAG_MFP_64HZ     0x04  /*  MFP is running at 64 Hz */
#define MCP7940N_FLAG_MFP_04KHZ    0x01  /*  MFP is running at 4 KHz */
#define MCP7940N_FLAG_MFP_8KHZ     0x02  /*  MFP is running at 8 KHz */
#define MCP7940N_FLAG_MFP_32KHZ    0x03  /*  MFP is running at 32 KHz */

////////////////////////////////////
// Flags for alarm control register.

#define MCP7940N_FLAG_ALMx_POL     0x80  /* Polarity of MFP on alarm */
#define MCP7940N_FLAG_ALMxC_SEC    0x00  /*  ALARM compare on SEC */
#define MCP7940N_FLAG_ALMxC_MIN    0x10  /*  ALARM compare on MIN */
#define MCP7940N_FLAG_ALMxC_HR     0x20  /*  ALARM compare on HOUR */
#define MCP7940N_FLAG_ALMxC_DAY    0x30  /*  ALARM compare on DAY */
#define MCP7940N_FLAG_ALMxC_DAT    0x40  /*  ALARM compare on DATE */
#define MCP7940N_FLAG_ALMxC_ALL    0x70  /*  ALARM compare on all parameters */
#define MCP7940N_FLAG_ALMx_IF      0x08  /*  MASK of the ALARM_IF */

////////////////////////////////////
// Various flags .

#define MCP7940N_FLAG_OSC_ON          0x20  /* State of the oscillator */

///////////////////////////
// Buffers.
  
#define RTC_MCP7940N_MAX_TRANSMIT_BUFFSER_SIZE 16

///////////////////////////
// Descriptor of RTC itself.

typedef uintptr_t RTC_MCP7940N_DriverHandle;
typedef uintptr_t RTC_MCP7940N_BufferHandle;

typedef enum RTC_MCP7940N_State {
  // No tasks needs to be performed.
  RTC_MCP7940N_STATE_NONE = 0,
  // RTC encountered an error.
  RTC_MCP7940N_STATE_ERROR,
  // Check status after I2C communication was requested.
  RTC_MCP7940N_STATE_I2C_STAUS_CHECK,
} RTC_MCP7940N_State;

typedef enum RTC_MCP7940N_NextTask {
  RTC_MCP7940N_TASK_NONE = 0,
  RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_ENABLE,
  RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_DISABLE,
  RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_STATUS,
  RTC_MCP7940N_TASK_BATTERY_UPDATE_ENABLE,
  RTC_MCP7940N_TASK_BATTERY_UPDATE_DISABLE,
  RTC_MCP7940N_TASK_BATTERY_UPDATE_STATUS,
  RTC_MCP7940N_TASK_DATE_TIME_CONVERT_BCD,
} RTC_MCP7940N_NextTask;

typedef struct RTC_MCP7940N_DateTime {
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  uint8_t day_of_week;
  uint8_t day;
  uint8_t month;
  uint8_t year;
} RTC_MCP7940N_DateTime;

typedef struct RTC_MCP7940N {
  // Current state of state machine.
  RTC_MCP7940N_State state;

  // Next task to be performed instead of going to a free state.
  RTC_MCP7940N_NextTask next_task;

  // I2C bus related handles.
  RTC_MCP7940N_DriverHandle i2c_handle;
  RTC_MCP7940N_BufferHandle i2c_buffer_handle;

  // Buffer to be transmitted,
  uint8_t transmit_buffer[RTC_MCP7940N_MAX_TRANSMIT_BUFFSER_SIZE];

  // Values used by intermediate tasks.
  // Must never be used externally.
  struct {
    uint8_t register_value;
    RTC_MCP7940N_DateTime* date_time;
    bool* return_status;
  } _private;
} RTC_MCP7940N;

// Initialize MCP7940N RTC descriptor at the given I2C module index.
//
// Will open all needed bus descriptors, initialize state machine.
//
// On success returns truth.
bool RTC_MCP7940N_Initialize(RTC_MCP7940N* rtc,
                             SYS_MODULE_INDEX i2c_module_index);

// Perform all MCP7940N related tasks.
void RTC_MCP7940N_Tasks(RTC_MCP7940N* rtc);

// Check whether RTC module is busy with any tasks.
bool RTC_MCP7940N_IsBusy(RTC_MCP7940N* rtc);

// Set current date and time.
//
// NOTE: date_time->day_of_week is ignored.
void RTC_MCP7940N_WriteDateAndTime(RTC_MCP7940N* rtc,
                                   const RTC_MCP7940N_DateTime* date_time);

// Read current date and time.
//
// The result is stored in date_time and can be accessed after
// RTC becomes free from tasks.
void RTC_MCP7940N_ReadDateAndTime(RTC_MCP7940N* rtc,
                                  RTC_MCP7940N_DateTime* date_time);

// Schedule task for reading given register value.
//
// The value will be stored in register_value and available after
// RTC becomes free from tasks.
void RTC_MCP7940N_ReadRegister(RTC_MCP7940N* rtc,
                               uint8_t register_address,
                               uint8_t* register_value);

// Set enabled bit on the oscillator.
void RTC_MCP7940N_EnableOscillator(RTC_MCP7940N* rtc, bool enable);

// Check whether oscillator is enabled.
void RTC_MCP7940N_OscillatorStatus(RTC_MCP7940N* rtc, bool* enabled);

// Set enabled bit on the battery backup.
void RTC_MCP7940N_EnableBatteryBackup(RTC_MCP7940N* rtc, bool enable);

// Check whether battery backup is enabled.
void RTC_MCP7940N_BatteryBackupStatus(RTC_MCP7940N* rtc, bool* enabled);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // _RTC_MCP7940N_H
