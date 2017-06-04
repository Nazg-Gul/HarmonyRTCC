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

#include "rtc_mcp7940n.h"

#include "system_definitions.h"

////////////////////////////////////////////////////////////////////////////////
// Handy compatibility/abstraction macros and types.

#define ERROR_MESSAGE(message) \
    SYS_CONSOLE_MESSAGE(RTC_MCP7940N_LOG_PREFIX message)

#define DEBUG_MESSAGE(message) \
    SYS_DEBUG_MESSAGE(0, "[DEBUG] " RTC_MCP7940N_LOG_PREFIX message)

#define DEBUG_PRINT(message, ...) \
    SYS_DEBUG_PRINT(0, "[DEBUG] " RTC_MCP7940N_LOG_PREFIX message, ##__VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////
// Internal routines.

static uint8_t convertToBCD(uint8_t decimal) {
  return (decimal / 10) << 4 | (decimal % 10);
}

static uint8_t convertFromBCD(uint8_t bcd) {
  uint8_t byte_msb = 0;
  uint8_t byte_lsb = 0;
  byte_msb = (bcd & 0b11110000) >> 4;
  byte_lsb = (bcd & 0b00001111);
  return ((byte_msb*10) + byte_lsb);
}

static DRV_I2C_BUFFER_EVENT getI2CTransferStatus(RTC_MCP7940N* rtc) {
  return DRV_I2C_TransferStatusGet(rtc->i2c_handle, rtc->i2c_buffer_handle);
}

static bool checkI2CBufferReadyForTransmit(RTC_MCP7940N* rtc) {
  return (rtc->i2c_buffer_handle == (DRV_I2C_BUFFER_HANDLE) NULL) || 
         (getI2CTransferStatus(rtc) == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
         (getI2CTransferStatus(rtc) == DRV_I2C_BUFFER_EVENT_ERROR);
}

static bool performI2CTransmit(RTC_MCP7940N* rtc, size_t num_bytes) {
  if (!checkI2CBufferReadyForTransmit(rtc)) {
    ERROR_MESSAGE("Unable to perform I2C transmittance.\r\n");
    rtc->state = RTC_MCP7940N_STATE_ERROR;
    return false;
  }
  DEBUG_PRINT("Performing transmittance of %d bytes.\r\n", num_bytes);
  // TODO(sergey): For some reason we need to shift address here...
  rtc->i2c_buffer_handle = DRV_I2C_Transmit(rtc->i2c_handle,
                                            (MCP7940N_I2C_ADDRESS << 1),
                                            &rtc->transmit_buffer[0],
                                            num_bytes,
                                            NULL);
  if (rtc->i2c_buffer_handle == NULL) {
    ERROR_MESSAGE("I2C transmit returned invalid handle.\r\n");
    rtc->state = RTC_MCP7940N_STATE_ERROR;
    return false;
  }
  rtc->state = RTC_MCP7940N_STATE_I2C_STAUS_CHECK;
  return true;
}

static bool performI2CTransmitThenReceive(RTC_MCP7940N* rtc,
                                          size_t num_bytes_send,
                                          uint8_t* receive_buffer,
                                          size_t num_bytes_receive) {
  if (!checkI2CBufferReadyForTransmit(rtc)) {
    ERROR_MESSAGE("Unable to perform I2C transmittance.\r\n");
    rtc->state = RTC_MCP7940N_STATE_ERROR;
    return false;
  }
  DEBUG_PRINT("Performing transmittance of %d bytes, "
              "followed with receiving %d bytes\r\n",
               num_bytes_send, num_bytes_receive);
  // TODO(sergey): For some reason we need to shift address here...
  rtc->i2c_buffer_handle = DRV_I2C_TransmitThenReceive(
      rtc->i2c_handle,
      (MCP7940N_I2C_ADDRESS << 1),
      &rtc->transmit_buffer[0], num_bytes_send,
      receive_buffer, num_bytes_receive,
      NULL);
  if (rtc->i2c_buffer_handle == NULL) {
    ERROR_MESSAGE("I2C transmit+receive returned invalid handle.\r\n");
    rtc->state = RTC_MCP7940N_STATE_ERROR;
    return false;
  }
  rtc->state = RTC_MCP7940N_STATE_I2C_STAUS_CHECK;
  return true;
}

static void oscillatorUpdateBits(RTC_MCP7940N* rtc) {
  DEBUG_PRINT("Register value before updating oscillator: %d.\r\n",
              rtc->register_value);
  rtc->transmit_buffer[0] = MCP7940N_REG_ADDR_SECONDS;
  if (rtc->next_task == RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_ENABLE) {
    rtc->transmit_buffer[1] = rtc->register_value | 
                              MCP7940N_FLAG_START_OSCILLATOR;
  } else {
    rtc->transmit_buffer[1] = rtc->register_value &
                              (~MCP7940N_FLAG_START_OSCILLATOR);
  }
  performI2CTransmit(rtc, 2);
}

static void batteryUpdateBits(RTC_MCP7940N* rtc) {
  DEBUG_PRINT("Register value before updating battery: %d.\r\n",
              rtc->register_value);
  rtc->transmit_buffer[0] = MCP7940N_REG_ADDR_DAY_OF_WEEK;
  if (rtc->next_task == RTC_MCP7940N_TASK_BATTERY_UPDATE_ENABLE) {
    rtc->transmit_buffer[1] = rtc->register_value | 
                              MCP7940N_FLAG_BATTERY_ENABLE;
  } else {
    rtc->transmit_buffer[1] = rtc->register_value &
                              (~MCP7940N_FLAG_BATTERY_ENABLE);
  }
  performI2CTransmit(rtc, 2);
}

static void dateTimeConvertBCD(RTC_MCP7940N* rtc) {
  rtc->date_time.seconds = convertFromBCD(rtc->date_time.seconds & 0x7f);
  rtc->date_time.minutes = convertFromBCD(rtc->date_time.minutes & 0x7f);
  rtc->date_time.hours = convertFromBCD(rtc->date_time.hours & 0x3f);

  rtc->date_time.day_of_week = convertFromBCD(rtc->date_time.day_of_week & 0x7);
  rtc->date_time.day = convertFromBCD(rtc->date_time.day & 0xf);
  rtc->date_time.month = convertFromBCD(rtc->date_time.month & 0xf);
  rtc->date_time.year = convertFromBCD(rtc->date_time.year & 0xff);
}

static void checkI2CStatus(RTC_MCP7940N* rtc) {
  DRV_I2C_BUFFER_EVENT status = getI2CTransferStatus(rtc);
  switch (status) {
    case DRV_I2C_BUFFER_EVENT_COMPLETE:
      DEBUG_MESSAGE("I2C transaction finished.\r\n");
      switch (rtc->next_task) {
        case RTC_MCP7940N_TASK_NONE:
          rtc->state = RTC_MCP7940N_STATE_NONE;
          break;
        case RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_ENABLE:
        case RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_DISABLE:
          // NOTE: Update next_task AFTER the function.
          oscillatorUpdateBits(rtc);
          rtc->next_task = RTC_MCP7940N_TASK_NONE;
          break;
        case RTC_MCP7940N_TASK_BATTERY_UPDATE_ENABLE:
        case RTC_MCP7940N_TASK_BATTERY_UPDATE_DISABLE:
          // NOTE: Update next_task AFTER the function.
          batteryUpdateBits(rtc);
          rtc->next_task = RTC_MCP7940N_TASK_NONE;
          break;
        case RTC_MCP7940N_TASK_DATE_TIME_CONVERT_BCD:
          rtc->next_task = RTC_MCP7940N_TASK_NONE;
          rtc->state = RTC_MCP7940N_STATE_NONE;
          dateTimeConvertBCD(rtc);
          break;
      }
      break;
    case DRV_I2C_BUFFER_EVENT_ERROR:
      ERROR_MESSAGE("Error detected during I2C transaction.\r\n");
      rtc->state = RTC_MCP7940N_STATE_ERROR;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Public API.

bool RTC_MCP7940N_Initialize(RTC_MCP7940N* rtc, 
                             SYS_MODULE_INDEX i2c_module_index) {
  rtc->i2c_handle = DRV_I2C_Open(i2c_module_index, DRV_IO_INTENT_READWRITE);
  if (rtc->i2c_handle == DRV_HANDLE_INVALID) {
    ERROR_MESSAGE("Error opening I2C bus handle.\r\n");
    rtc->state = RTC_MCP7940N_STATE_ERROR;
    return false;
  }
  rtc->state = RTC_MCP7940N_STATE_NONE;
  rtc->next_task = RTC_MCP7940N_TASK_NONE;
  DEBUG_MESSAGE("New RTC handle is initialized.\r\n");
  return true;
}

void RTC_MCP7940N_Tasks(RTC_MCP7940N* rtc) {
  switch (rtc->state) {
    case RTC_MCP7940N_STATE_NONE:
      // Nothing to do, pass.
      break;
    case RTC_MCP7940N_STATE_ERROR:
      // TODO(sergey): Report some extra error message?
      break;
    case RTC_MCP7940N_STATE_I2C_STAUS_CHECK:
      checkI2CStatus(rtc);
      break;
  }
}

bool RTC_MCP7940N_IsBusy(RTC_MCP7940N* rtc) {
  return !(rtc->state == RTC_MCP7940N_STATE_NONE ||
           rtc->state == RTC_MCP7940N_STATE_ERROR);
}

void RTC_MCP7940N_WriteDateTime(RTC_MCP7940N* rtc,
                                RTC_MCP7940N_DateTime* date_time) {
  DEBUG_MESSAGE("Begin transmitting date and time to RTC.\r\n");
  // Prepare buffer to be transmitted.
  rtc->transmit_buffer[0] = MCP7940N_REG_ADDR_SECONDS;
  // Store date/time to the buffer.
  //
  // NOTE: We use arbitrary value for day of week.
  rtc->transmit_buffer[1] = convertToBCD(date_time->seconds);
  rtc->transmit_buffer[2] = convertToBCD(date_time->minutes);
  rtc->transmit_buffer[3] = convertToBCD(date_time->hours);
  rtc->transmit_buffer[4] = convertToBCD(6);
  rtc->transmit_buffer[5] = convertToBCD(date_time->day);
  rtc->transmit_buffer[6] = convertToBCD(date_time->month);
  rtc->transmit_buffer[7] = convertToBCD(date_time->year);
  // Schedule transmittance.
  performI2CTransmit(rtc, 8);
}

void RTC_MCP7940N_ReadDateAndTime(RTC_MCP7940N* rtc) {
  DEBUG_PRINT("Begin sequence to read current date and time\r\n");
  // Prepare transmittance buffer.
  rtc->transmit_buffer[0] = MCP7940N_REG_ADDR_SECONDS;
  // Schedule receive.
  rtc->next_task = RTC_MCP7940N_TASK_DATE_TIME_CONVERT_BCD;
  performI2CTransmitThenReceive(rtc,
                                1,
                                (uint8_t*)&rtc->date_time, 
                                sizeof(rtc->date_time));
}

void RTC_MCP7940N_ReadRegister(RTC_MCP7940N* rtc, uint8_t register_address) {
  DEBUG_PRINT("Begin receiving register %x from RTC.\r\n", register_address);
  rtc->register_value = 0;
  // Prepare transmittance buffer.
  rtc->transmit_buffer[0] = register_address;
  // Schedule receive.
  performI2CTransmitThenReceive(rtc, 1, &rtc->register_value, 1);
}

void RTC_MCP7940N_EnableOscillator(RTC_MCP7940N* rtc, bool enable) {
  DEBUG_PRINT("Begin sequence to set oscillator status to %s.\r\n",
              enable ? "ENABLED" : "DISABLED");
  // Prepare transmittance buffer.
  rtc->transmit_buffer[0] = MCP7940N_REG_ADDR_SECONDS;
  // Schedule receive.
  if (enable) {
    rtc->next_task = RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_ENABLE;
  } else {
    rtc->next_task = RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_DISABLE;
  }
  performI2CTransmitThenReceive(rtc, 1, &rtc->register_value, 1);
}

void RTC_MCP7940N_EnableBatterBackup(RTC_MCP7940N* rtc, bool enable) {
  DEBUG_PRINT("Begin sequence to set battery backup to %s.\r\n",
              enable ? "ENABLED" : "DISABLED");
  // Prepare transmittance buffer.
  rtc->transmit_buffer[0] = MCP7940N_REG_ADDR_DAY_OF_WEEK;
  // Schedule receive.
  if (enable) {
    rtc->next_task = RTC_MCP7940N_TASK_BATTERY_UPDATE_ENABLE;
  } else {
    rtc->next_task = RTC_MCP7940N_TASK_BATTERY_UPDATE_DISABLE;
  }
  performI2CTransmitThenReceive(rtc, 1, &rtc->register_value, 1);
}
