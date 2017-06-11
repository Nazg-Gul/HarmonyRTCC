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

#define RTC_MCP7940N_LOG_PREFIX "MCP7940N: "

#define DEBUG_LEVEL 0

#define ERROR_MESSAGE(message) \
    SYS_CONSOLE_MESSAGE(RTC_MCP7940N_LOG_PREFIX message)

#define DEBUG_MESSAGE(message) \
    SYS_DEBUG_MESSAGE(DEBUG_LEVEL, "[DEBUG] " RTC_MCP7940N_LOG_PREFIX message)

#define DEBUG_PRINT(format, ...) \
    SYS_DEBUG_PRINT(DEBUG_LEVEL, "[DEBUG] " RTC_MCP7940N_LOG_PREFIX format, ##__VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////
// Mathematical utilities
//
// TODO(sergey): Consider moving them to some more generic header file..

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

////////////////////////////////////////////////////////////////////////////////
// Debug helpers.

#ifdef SYS_CMD_REMAP_SYS_DEBUG_MESSAGE
static void debugPrintRegisters(uint8_t* registers, uint8_t num_registers) {
  uint8_t current_register = 0;
  while (current_register < num_registers) {
    uint8_t i;
    SYS_DEBUG_PRINT(DEBUG_LEVEL, "%02x |", current_register);
    for (i = 0; i < 8 && current_register < num_registers; ++i) {
      SYS_DEBUG_PRINT(DEBUG_LEVEL, " %02x", registers[current_register]);
      current_register++;
    }
    SYS_DEBUG_MESSAGE(DEBUG_LEVEL, "\r\n");
  }
}
#endif

////////////////////////////////////////////////////////////////////////////////
// Low-level I2C communication helpers.

// Helper function to get main transmit/receive buffer status for given
// RTC handle.
static inline DRV_I2C_BUFFER_EVENT i2c_transferStatusGet(RTC_MCP7940N* rtc) {
  return DRV_I2C_TransferStatusGet(rtc->i2c_handle, rtc->i2c_buffer_handle);
}

// Check whether we are ready to transmit data to RTC.
static bool i2c_checkReadyForTransmit(RTC_MCP7940N* rtc) {
  if (rtc->i2c_buffer_handle == NULL) {
    return true;
  }
  const DRV_I2C_BUFFER_EVENT status = i2c_transferStatusGet(rtc);
  return (status == DRV_I2C_BUFFER_EVENT_COMPLETE) ||
         (status == DRV_I2C_BUFFER_EVENT_ERROR);
}

// Transmit given number of bytes to the RTC.
static bool i2c_transmit(RTC_MCP7940N* rtc,
                         uint8_t* transmit_buffer,
                         size_t num_bytes) {
  if (!i2c_checkReadyForTransmit(rtc)) {
    ERROR_MESSAGE("Unable to perform I2C transmittance.\r\n");
    rtc->state = RTC_MCP7940N_STATE_ERROR;
    return false;
  }
  DEBUG_PRINT("Transmitting %d bytes.\r\n", num_bytes);
  // TODO(sergey): For some reason we need to shift address here...
  rtc->i2c_buffer_handle = DRV_I2C_Transmit(rtc->i2c_handle,
                                            (MCP7940N_I2C_ADDRESS << 1),
                                            transmit_buffer,
                                            num_bytes,
                                            NULL);
  if (rtc->i2c_buffer_handle == NULL) {
    ERROR_MESSAGE("I2C Transmit returned invalid handle.\r\n");
    rtc->state = RTC_MCP7940N_STATE_ERROR;
    return false;
  }
  rtc->state = RTC_MCP7940N_STATE_I2C_STAUS_CHECK;
  return true;
}

// Transmit given number of bytes to RTC, then receive given number of bytes
// back from RTC.
static bool i2c_transmitThenReceive(RTC_MCP7940N* rtc,
                                    uint8_t* transmit_buffer,
                                    size_t num_bytes_transmit,
                                    uint8_t* receive_buffer,
                                    size_t num_bytes_receive) {
  if (!i2c_checkReadyForTransmit(rtc)) {
    ERROR_MESSAGE("Unable to perform I2C transmittance.\r\n");
    rtc->state = RTC_MCP7940N_STATE_ERROR;
    return false;
  }
  DEBUG_PRINT("Transmitting %d bytes, then receiving %d bytes.\r\n",
              num_bytes_transmit, num_bytes_receive);
  // TODO(sergey): For some reason we need to shift address here...
  rtc->i2c_buffer_handle = DRV_I2C_TransmitThenReceive(
      rtc->i2c_handle,
      (MCP7940N_I2C_ADDRESS << 1),
      transmit_buffer, num_bytes_transmit,
      receive_buffer, num_bytes_receive,
      NULL);
  if (rtc->i2c_buffer_handle == NULL) {
    ERROR_MESSAGE("I2C TransmitThenReceive returned invalid handle.\r\n");
    rtc->state = RTC_MCP7940N_STATE_ERROR;
    return false;
  }
  rtc->state = RTC_MCP7940N_STATE_I2C_STAUS_CHECK;
  return true;
}

// Helper function to receive value of a single register.
//
// The buffer is supposed to be 1 byte:
// - First byte is the address of the register.
static bool i2c_transmitReadRegister(RTC_MCP7940N* rtc,
                                     uint8_t* transmit_buffer,
                                     uint8_t* receive_buffer,
                                     size_t num_bytes_receive) {
  DEBUG_PRINT("Receiving value of %d registers staring at 0x%02x.\r\n",
              num_bytes_receive, transmit_buffer[0]);
  return i2c_transmitThenReceive(rtc,
                                 transmit_buffer, 1,
                                 receive_buffer, num_bytes_receive);
}

// Helper function to update value of a single register.
//
// The buffer is supposed to be 2 bytes:
// - First byte is the address of the register.
// - Second byte is the new register value.
static bool i2c_transmitUpdateSingleRegister(RTC_MCP7940N* rtc,
                                             uint8_t* transmit_buffer) {
  DEBUG_PRINT("Updating value of register 0x%02x to value of 0x%02x.\r\n",
               transmit_buffer[0], transmit_buffer[1]);
  return i2c_transmit(rtc, transmit_buffer, 2);
}

////////////////////////////////////////////////////////////////////////////////
// Task-related callbacks.
//
// Those functions are used as next-task, which are being invoked after initial
// data transfer is over.

// Invoked after oscillator status register is read. Updates the register value
// based on the task type: will either enable or disable oscillator.
static void task_oscillator_enableOrDisable(RTC_MCP7940N* rtc) {
  uint8_t register_value = rtc->_private.oscillator_bits.current_register_value;
  DEBUG_PRINT("Register value before updating oscillator: 0x%02x.\r\n",
              register_value);
  // Update register value based on the next task.
  if (rtc->next_task == RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_ENABLE) {
    register_value |= MCP7940N_FLAG_START_OSCILLATOR;
  } else {
    SYS_ASSERT(rtc->next_task == RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_DISABLE,
               "Unexpected value of rtc->next_task");
    register_value &= ~MCP7940N_FLAG_START_OSCILLATOR;
  }
  // Transmit new register value to the RTC.
  uint8_t* transmit_buffer = rtc->_private.oscillator_bits.transmit_buffer;
  transmit_buffer[0] = MCP7940N_REG_ADDR_SECONDS;
  transmit_buffer[1] = register_value;
  i2c_transmitUpdateSingleRegister(rtc, transmit_buffer);
  // Clear state machine.
  rtc->next_task = RTC_MCP7940N_TASK_NONE;
}

static void task_oscillator_postStatusReceive(RTC_MCP7940N* rtc) {
  const uint8_t register_value =
      rtc->_private.oscillator_status.current_register_value;
  DEBUG_PRINT("Fetched register value: 0x%02x.\r\n", register_value);
  const bool enabled = (register_value & MCP7940N_FLAG_START_OSCILLATOR) != 0;
  *rtc->_private.oscillator_status.return_status_ptr = enabled;
  // Clear state machine.
  rtc->next_task = RTC_MCP7940N_TASK_NONE;
}

static void task_battery_enableOrDisable(RTC_MCP7940N* rtc) {
  uint8_t register_value = rtc->_private.battery_bits.current_register_value;
  DEBUG_PRINT("Register value before updating battery: 0x%02x.\r\n",
              register_value);
  // Update register value based on the next task.
  if (rtc->next_task == RTC_MCP7940N_TASK_BATTERY_UPDATE_ENABLE) {
    register_value |= MCP7940N_FLAG_BATTERY_ENABLE;
  } else {
    SYS_ASSERT(rtc->next_task == RTC_MCP7940N_TASK_BATTERY_UPDATE_DISABLE,
               "Unexpected value of rtc->next_task");
    register_value &= ~MCP7940N_FLAG_BATTERY_ENABLE;
  }
  // Transmit new register value to the RTC.
  uint8_t* transmit_buffer = rtc->_private.battery_bits.transmit_buffer;
  transmit_buffer[0] = MCP7940N_REG_ADDR_DAY_OF_WEEK;
  transmit_buffer[1] = register_value;
  i2c_transmitUpdateSingleRegister(rtc, transmit_buffer);
  // Clear state machine.
  rtc->next_task = RTC_MCP7940N_TASK_NONE;
}

static void task_battery_postStatusReceive(RTC_MCP7940N* rtc) {
  const uint8_t register_value =
      rtc->_private.battery_status.current_register_value;
  DEBUG_PRINT("Fetched register value: 0x%02x.\r\n", register_value);
  const bool enabled = (register_value & MCP7940N_FLAG_BATTERY_ENABLE) != 0;
  *rtc->_private.battery_status.return_status_ptr = enabled;
  // Clear state machine.
  rtc->next_task = RTC_MCP7940N_TASK_NONE;
}

static void task_datetime_convertToBCDAndTransmit(RTC_MCP7940N* rtc) {
  const uint8_t num_registers = sizeof(RTC_MCP7940N_DateTime);
  const RTC_MCP7940N_DateTime *date_time = &rtc->_private.date_time_write.date_time;
  uint8_t *reg = rtc->_private.date_time_write.register_storage;
#ifdef SYS_CMD_REMAP_SYS_DEBUG_MESSAGE
  DEBUG_MESSAGE("Fetched register values before updating:\r\n");
  debugPrintRegisters(reg + 1, num_registers);
#endif
  reg[0] = MCP7940N_REG_ADDR_SECONDS;
  reg[1] = (reg[1] & ~0x7f) | convertToBCD(date_time->seconds);
  reg[2] = (reg[2] & ~0x7f) | convertToBCD(date_time->minutes);
  reg[3] = (reg[3] & ~0x3f) | convertToBCD(date_time->hours);
  reg[4] = (reg[4] & ~0x7) | convertToBCD(date_time->day_of_week);
  reg[5] = (reg[5] & ~0x3f) | convertToBCD(date_time->day);
  reg[6] = (reg[6] & ~0xf) | convertToBCD(date_time->month);
  reg[7] = (reg[7] & ~0xff) | convertToBCD(date_time->year);
  i2c_transmit(rtc, reg, num_registers + 1);
  // Clear state machine.
  rtc->next_task = RTC_MCP7940N_TASK_NONE;
}

static void task_datetime_convertFromBCD(RTC_MCP7940N* rtc) {
  RTC_MCP7940N_DateTime* date_time = rtc->_private.date_time_read.date_time_ptr;
  date_time->seconds = convertFromBCD(date_time->seconds & 0x7f);
  date_time->minutes = convertFromBCD(date_time->minutes & 0x7f);
  date_time->hours = convertFromBCD(date_time->hours & 0x3f);
  date_time->day_of_week = convertFromBCD(date_time->day_of_week & 0x7);
  date_time->day = convertFromBCD(date_time->day & 0x3f);
  date_time->month = convertFromBCD(date_time->month & 0xf);
  date_time->year = convertFromBCD(date_time->year & 0xff);
  // Clear state machine.
  rtc->state = RTC_MCP7940N_STATE_NONE;
  rtc->next_task = RTC_MCP7940N_TASK_NONE;
}

// Check status of I2C bus and invoke next tasks when needed.
static void i2c_taskCheckStatus(RTC_MCP7940N* rtc) {
  const DRV_I2C_BUFFER_EVENT status = i2c_transferStatusGet(rtc);
  switch (status) {
    case DRV_I2C_BUFFER_EVENT_COMPLETE:
      DEBUG_MESSAGE("I2C transaction finished.\r\n");
      switch (rtc->next_task) {
        case RTC_MCP7940N_TASK_NONE:
          rtc->state = RTC_MCP7940N_STATE_NONE;
          break;
        case RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_ENABLE:
        case RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_DISABLE:
          task_oscillator_enableOrDisable(rtc);
          break;
        case RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_STATUS:
          task_oscillator_postStatusReceive(rtc);
          break;
        case RTC_MCP7940N_TASK_BATTERY_UPDATE_ENABLE:
        case RTC_MCP7940N_TASK_BATTERY_UPDATE_DISABLE:
          task_battery_enableOrDisable(rtc);
          break;
        case RTC_MCP7940N_TASK_BATTERY_UPDATE_STATUS:
          rtc->next_task = RTC_MCP7940N_TASK_NONE;
          task_battery_postStatusReceive(rtc);
          break;
        case RTC_MCP7940N_TASK_DATE_TIME_CONVERT_BCD:
          task_datetime_convertFromBCD(rtc);
          break;
        case RTC_MCP7940N_TASK_DATE_TIME_UPDATE_AND_TRANSMIT:
          task_datetime_convertToBCDAndTransmit(rtc);
          break;
      }
      break;
    case DRV_I2C_BUFFER_EVENT_ERROR:
      ERROR_MESSAGE("Error detected during I2C transaction.\r\n");
      rtc->state = RTC_MCP7940N_STATE_ERROR;
      break;
    default:
      // Nothing to do.
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Public API.

bool RTC_MCP7940N_Initialize(RTC_MCP7940N* rtc,
                             SYS_MODULE_INDEX i2c_module_index) {
  // TODO(sergey): Ignore for release builds to save CPU ticks?
  memset(rtc, 0, sizeof(*rtc));
  rtc->i2c_handle = DRV_I2C_Open(i2c_module_index, DRV_IO_INTENT_READWRITE);
  if (rtc->i2c_handle == DRV_HANDLE_INVALID) {
    ERROR_MESSAGE("Error opening I2C bus handle.\r\n");
    rtc->state = RTC_MCP7940N_STATE_ERROR;
    return false;
  }
  rtc->state = RTC_MCP7940N_STATE_NONE;
  rtc->next_task = RTC_MCP7940N_TASK_NONE;
  rtc->i2c_buffer_handle = NULL;
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
      i2c_taskCheckStatus(rtc);
      break;
  }
}

bool RTC_MCP7940N_IsBusy(RTC_MCP7940N* rtc) {
  return !(rtc->state == RTC_MCP7940N_STATE_NONE ||
           rtc->state == RTC_MCP7940N_STATE_ERROR);
}

void RTC_MCP7940N_WriteDateAndTime(RTC_MCP7940N* rtc,
                                   const RTC_MCP7940N_DateTime* date_time) {
  DEBUG_MESSAGE("Begin transmitting date and time to RTC.\r\n");
  // We should preserve non-date-time bits intact, so we first read old state
  // and then write an updated one.
  rtc->next_task = RTC_MCP7940N_TASK_DATE_TIME_UPDATE_AND_TRANSMIT;
  // Make a copy of date time to be sent to RTC.
  memcpy(&rtc->_private.date_time_write.date_time,
         date_time,
         sizeof(rtc->_private.date_time_write.date_time));
  RTC_MCP7940N_ReadNumRegisters(
      rtc,
      rtc->_private.date_time_write.register_storage + 1,
      sizeof(rtc->_private.date_time_write.date_time));
}

void RTC_MCP7940N_ReadDateAndTime(RTC_MCP7940N* rtc,
                                  RTC_MCP7940N_DateTime* date_time) {
  DEBUG_PRINT("Begin sequence to read current date and time\r\n");
  // Prepare transmittance buffer.
  uint8_t* transmit_buffer = rtc->_private.date_time_read.transmit_buffer;
  transmit_buffer[0] = MCP7940N_REG_ADDR_SECONDS;
  // Schedule receive.
  rtc->next_task = RTC_MCP7940N_TASK_DATE_TIME_CONVERT_BCD;
  rtc->_private.date_time_read.date_time_ptr = date_time;
  i2c_transmitReadRegister(rtc,
                           transmit_buffer,
                           (uint8_t*)date_time, sizeof(*date_time));
}

void RTC_MCP7940N_ReadRegister(RTC_MCP7940N* rtc,
                               uint8_t register_address,
                               uint8_t* register_value) {
  DEBUG_PRINT("Begin receiving register 0x%02x from RTC.\r\n", register_address);
  // Prepare transmittance buffer.
  uint8_t* transmit_buffer = rtc->_private.register_read.transmit_buffer;
  transmit_buffer[0] = register_address;
  // Schedule receive.
  i2c_transmitReadRegister(rtc, transmit_buffer, register_value, 1);
}

void RTC_MCP7940N_WriteRegister(RTC_MCP7940N* rtc,
                                uint8_t register_address,
                                uint8_t register_value) {
  DEBUG_PRINT("Begin writing register 0x%02x to RTC with value 0x%02x.\r\n", 
              register_address, register_value);
  // Prepare transmittance buffer.
  uint8_t* transmit_buffer = rtc->_private.register_write.transmit_buffer;
  transmit_buffer[0] = register_address;
  transmit_buffer[1] = register_value;
  // Schedule transmit.
  i2c_transmitUpdateSingleRegister(rtc, transmit_buffer);
}

void RTC_MCP7940N_ReadNumRegisters(RTC_MCP7940N* rtc,
                                   uint8_t* register_storage,
                                   uint8_t num_registers) {
  SYS_ASSERT(num_registers <= RTC_MCP7940N_NUM_REGISTERS,
             "Attempt to read too many registers");
  DEBUG_PRINT("Begin reading %d registers.\r\n", num_registers);
  uint8_t* transmit_buffer = rtc->_private.register_num_read.transmit_buffer;
  transmit_buffer[0] = MCP7940N_REG_ADDR_SECONDS;
  // Schedule receive.
  i2c_transmitReadRegister(rtc,
                           transmit_buffer,
                           register_storage, num_registers);
}

void RTC_MCP7940N_WriteNumRegisters(RTC_MCP7940N* rtc,
                                    const uint8_t* register_storage,
                                    uint8_t num_registers) {
  SYS_ASSERT(num_registers <= RTC_MCP7940N_NUM_REGISTERS,
             "Attempt to read too many registers");
  DEBUG_PRINT("Begin writing %d registers.\r\n", num_registers);
  uint8_t* transmit_buffer = rtc->_private.register_num_write.transmit_buffer;
  transmit_buffer[0] = MCP7940N_REG_ADDR_SECONDS;
  memcpy(&transmit_buffer[1], register_storage, num_registers);
  i2c_transmit(rtc, transmit_buffer, (num_registers + 1));
}

void RTC_MCP7940N_EnableOscillator(RTC_MCP7940N* rtc, bool enable) {
  DEBUG_PRINT("Begin sequence to set oscillator status to %s.\r\n",
              enable ? "ENABLED" : "DISABLED");
  // Prepare transmittance buffer.
  uint8_t* transmit_buffer = rtc->_private.oscillator_bits.transmit_buffer;
  transmit_buffer[0] = MCP7940N_REG_ADDR_SECONDS;
  rtc->next_task = enable ? RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_ENABLE
                          : RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_DISABLE;
  // Schedule receive.
  i2c_transmitReadRegister(
      rtc,
      transmit_buffer,
      &rtc->_private.oscillator_bits.current_register_value, 1);
}

void RTC_MCP7940N_OscillatorStatus(RTC_MCP7940N* rtc, bool* enabled) {
  DEBUG_MESSAGE("Begin sequence to check whether oscillator is enabled.\r\n");
  uint8_t* transmit_buffer = rtc->_private.oscillator_status.transmit_buffer;
  transmit_buffer[0] = MCP7940N_REG_ADDR_SECONDS;
  rtc->next_task = RTC_MCP7940N_TASK_OSCILLATOR_UPDATE_STATUS;
  rtc->_private.oscillator_status.return_status_ptr = enabled;
  // Schedule receive.
  i2c_transmitReadRegister(
      rtc,
      transmit_buffer,
      &rtc->_private.oscillator_status.current_register_value, 1);
}

void RTC_MCP7940N_EnableBatteryBackup(RTC_MCP7940N* rtc, bool enable) {
  DEBUG_PRINT("Begin sequence to set battery backup to %s.\r\n",
              enable ? "ENABLED" : "DISABLED");
  // Prepare transmittance buffer.
  uint8_t* transmit_buffer = rtc->_private.battery_bits.transmit_buffer;
  transmit_buffer[0] = MCP7940N_REG_ADDR_DAY_OF_WEEK;
  rtc->next_task = enable ? RTC_MCP7940N_TASK_BATTERY_UPDATE_ENABLE
                          : RTC_MCP7940N_TASK_BATTERY_UPDATE_DISABLE;
  // Schedule receive.
  i2c_transmitReadRegister(
      rtc,
      transmit_buffer,
      &rtc->_private.battery_bits.current_register_value, 1);
}

void RTC_MCP7940N_BatteryBackupStatus(RTC_MCP7940N* rtc, bool* enabled) {
  DEBUG_MESSAGE("Begin sequence to check whether battery backup "
                "is enabled.\r\n");
  uint8_t* transmit_buffer = rtc->_private.battery_status.transmit_buffer;
  transmit_buffer[0] = MCP7940N_REG_ADDR_DAY_OF_WEEK;
  rtc->next_task = RTC_MCP7940N_TASK_BATTERY_UPDATE_STATUS;
  rtc->_private.battery_status.return_status_ptr = enabled;
  // Schedule receive.
  i2c_transmitReadRegister(
      rtc,
      transmit_buffer,
      &rtc->_private.battery_status.current_register_value, 1);
}
