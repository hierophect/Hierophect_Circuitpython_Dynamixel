# The MIT License (MIT)
#
# Copyright (c) 2020 Lucian Copeland
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
`dynamixel`
================================================================================

Circuitpython driver library for the Dynamixel series of servo motors from
Robotis.

Dynamixels are a series of smart actuators designed to form the connecting
joints on a robot or other mechanical structure. They utilize an addressed UART
bus system, allowing them to be daisy chained to one another with a minimum of
cabling. Dynamixels also contain an integrated controller for setting torque and
temperature limits, speed adjustment, continuous rotation mode, and other
features.

The AX series is supported by this library. Support for the RX and MX series is
likely but untested.

**Software and Dependencies:**

# * Adafruit's Circuitpython Releases: https://github.com/adafruit/circuitpython/releases
# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
# * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register

* Author(s):

    - Lucian Copeland (hierophect)
"""

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/hierophect/CircuitPython_dynamixel.git"

import time
from micropython import const

# Addresses
# EEPROM
DYN_REG_MODEL_NUMBER_L          = const(0x00)
DYN_REG_MODEL_NUMBER_H          = const(0x01)
DYN_REG_FIRMWARE_VER            = const(0x02)
DYN_REG_ID                      = const(0x03)
DYN_REG_BAUD                    = const(0x04)
DYN_REG_RETURN_DELAY            = const(0x05)
DYN_REG_CW_ANGLE_LIMIT_L        = const(0x06)
DYN_REG_CW_ANGLE_LIMIT_H        = const(0x07)
DYN_REG_CCW_ANGLE_LIMIT_L       = const(0x08)
DYN_REG_CCW_ANGLE_LIMIT_H       = const(0x09)
# -- Reserved = const(0x00)
DYN_REG_LIMIT_MAX_TEMP          = const(0x0B)
DYN_REG_LIMIT_MIN_VOLT          = const(0x0C)
DYN_REG_LIMIT_MAX_VOLT          = const(0x0D)
DYN_REG_MAX_TORQUE_L            = const(0x0E)
DYN_REG_MAX_TORQUE_H            = const(0x0F)
DYN_REG_STATUS_RETURN_LEVEL     = const(0x10)
DYN_REG_ALARM_LED               = const(0x11)
DYN_REG_ALARM_SHUTDOWN          = const(0x12)
# -- Reserved = const(0x00)
DYN_REG_DOWN_CALIB_L            = const(0x14)
DYN_REG_DOWN_CALIB_H            = const(0x15)
DYN_REG_UP_CALIB_L              = const(0x16)
DYN_REG_UP_CALIB_H              = const(0x17)

# RAM = const(0x00)
DYN_REG_TORQUE_ENABLE           = const(0x18)
DYN_REG_LED                     = const(0x19)
DYN_REG_CW_COMPLIANCE_MARGIN    = const(0x1A)
DYN_REG_CCW_COMPLIANCE_MARGIN   = const(0x1B)
DYN_REG_CW_COMPLIANCE_SLOPE     = const(0x1C)
DYN_REG_CCW_COMPLIANCE_SLOPE    = const(0x1D)
DYN_REG_GOAL_POSITION_L         = const(0x1E)
DYN_REG_GOAL_POSITION_H         = const(0x1F)
DYN_REG_MOVING_SPEED_L          = const(0x20)
DYN_REG_MOVING_SPEED_H          = const(0x21)
DYN_REG_TORQUE_LIMIT_L          = const(0x22)
DYN_REG_TORQUE_LIMIT_H          = const(0x23)
DYN_REG_PRESENT_POSITION_L      = const(0x24)
DYN_REG_PRESENT_POSITION_H      = const(0x25)
DYN_REG_PRESENT_SPEED_L         = const(0x26)
DYN_REG_PRESENT_SPEED_H         = const(0x27)
DYN_REG_PRESENT_LOAD_L          = const(0x28)
DYN_REG_PRESENT_LOAD_H          = const(0x29)
DYN_REG_PRESENT_VOLTAGE         = const(0x2A)
DYN_REG_PRESENT_TEMP            = const(0x2B)
DYN_REG_REGISTERED_INST         = const(0x2C)
# -- Reserved = const(0x00)
DYN_REG_MOVING                  = const(0x2E)
DYN_REG_LOCK                    = const(0x2F)
DYN_REG_PUNCH_L                 = const(0x30)
DYN_REG_PUNCH_H                 = const(0x31)

DYN_ERR_NONE                    = const(0x00)
DYN_ERR_VOLTAGE                 = const(0x01)
DYN_ERR_ANGLE                   = const(0x02)
DYN_ERR_OVERHEAT                = const(0x04)
DYN_ERR_RANGE                   = const(0x08)
DYN_ERR_CHECKSUM                = const(0x10)
DYN_ERR_OVERLOAD                = const(0x20)
DYN_ERR_INST                    = const(0x40)
DYN_ERR_INVALID                 = const(0x80)

DYN_INST_PING                   = const(0x01)
DYN_INST_READ                   = const(0x02)
DYN_INST_WRITE                  = const(0x03)
DYN_INST_REG_WRITE              = const(0x04)
DYN_INST_ACTION                 = const(0x05)
DYN_INST_RESET                  = const(0x06)
DYN_INST_SYNC_WRITE             = const(0x83)

DYN_BROADCAST_ID                = const(0xFE)

# Send packet structure:
# | 0xFF | 0xFF | ID | LEN | INST | PARAM_1-PARAM_N | CHECKSUM |
# Status Packet structure
# | 0xFF | 0xFF | ID | LEN | ERROR | VALUE_1-VALUE_N | CHECKSUM |

class Dynamixel:
    def __init__(self, uart, direction_pin):
        self._uart = uart
        self._dir = direction_pin
        # Disable RX when not explicitly reading
        self._dir.value = True
        self.last_error = DYN_ERR_INVALID

    # -----------------------
    # Dynamixel Instructions:
    # -----------------------

    def ping(self, dyn_id):
        length = 2
        inst = DYN_INST_PING
        checksum = (~(dyn_id+length+inst)+256) % 256
        data = [255,255,dyn_id,length,inst,checksum]
        array = bytes(data)
        # Write out the request
        self._uart.write(array)
        time.sleep(0.001) # Delay required to avoid premature opening of RX
        # Open the RX line and read data
        self._dir.value = False
        packet = self._uart.read(6)
        self._dir.value = True
        time.sleep(0.001)
        # Check if packet timed out and is empty
        if packet is None:
            raise RuntimeError("Could not find motor at supplied address")
        # Otherwise, return the error
        self.last_error = packet[4]
        return packet[4]

    def read_data(self, dyn_id, reg_addr, nbytes):
        length = 4
        inst = DYN_INST_READ
        checksum = (~(dyn_id+length+inst+reg_addr+nbytes)+256) % 256
        data = [255,255,dyn_id,length,inst,reg_addr,nbytes,checksum]
        array = bytes(data)
        # Write out the request
        self._uart.write(array)
        time.sleep(0.001) # Delay required to avoid premature opening of RX
        # Open the RX line and read data
        self._dir.value = False
        packet = self._uart.read(nbytes+6)
        self._dir.value = True
        time.sleep(0.001)
        # Check if packet timed out and is empty
        if packet is None:
            raise RuntimeError("Could not find motor at supplied address")
        self.last_error = packet[4]
        return packet

    def write_data(self, dyn_id, reg_addr, parameters):
        length = len(parameters) + 3
        inst = DYN_INST_WRITE
        checksum = (~(dyn_id+length+inst+reg_addr+sum(parameters))+256) % 256
        data = [255, 255, dyn_id, length, inst, reg_addr]
        data.extend(parameters)
        data.append(checksum)
        array = bytes(data)
        # Write the data
        self._uart.write(array)
        time.sleep(0.001) # Delay required to avoid premature opening of RX
        # Check for error only if this is not broadcast mode
        if dyn_id != DYN_BROADCAST_ID:
            # Open the RX line, but only return error
            self._dir.value = False
            packet = self._uart.read(6)
            self._dir.value = True
            time.sleep(0.001)
            # Check if packet timed out and is empty
            if packet is None:
                raise RuntimeError("Could not find motor at supplied address")
            # self._uart.reset_input_buffer()
            self.last_error = packet[4]
        else:
            self.last_error = DYN_ERR_INVALID

    def reg_write(self, dyn_id, reg_addr, parameters):
        length = len(parameters) + 3
        inst = DYN_INST_REG_WRITE
        checksum = (~(dyn_id+length+inst+reg_addr+sum(parameters))+256) % 256
        data = [255, 255, dyn_id, length, inst, reg_addr]
        data.extend(parameters)
        data.append(checksum)
        array = bytes(data)
        self._uart.write(array)
        time.sleep(0.001)
        if dyn_id != DYN_BROADCAST_ID:
            self._dir.value = False
            packet = self._uart.read(6)
            self._dir.value = True
            time.sleep(0.001)
            if packet is None:
                raise RuntimeError("Could not find motor at supplied address")
            self.last_error = packet[4]
        else:
            self.last_error = DYN_ERR_INVALID

    def action(self, dyn_id):
        length = 2
        inst = DYN_INST_ACTION
        checksum = (~(dyn_id+length+inst)+256) % 256
        data = [255,255,dyn_id,length,inst,checksum]
        array = bytes(data)
        self._uart.write(array)
        time.sleep(0.001)
        if dyn_id != DYN_BROADCAST_ID:
            self._dir.value = False
            packet = self._uart.read(6)
            self._dir.value = True
            time.sleep(0.001)
            if packet is None:
                raise RuntimeError("Could not find motor at supplied address")
            self.last_error = packet[4]
        else:
            self.last_error = DYN_ERR_INVALID


    def reset(self, dyn_id):
        length = 2
        inst = DYN_INST_RESET
        checksum = (~(dyn_id+length+inst)+256) % 256
        data = [255,255,dyn_id,length,inst,checksum]
        array = bytes(data)
        self._uart.write(array)
        time.sleep(0.001)
        # Value may adjust baudrate, so no error check.
        self.last_error = DYN_ERR_INVALID

    def sync_write(self, parameters):
        if not (isinstance(parameters[0], list)):
            raise ValueError("Sync parameter list must be multidimensional")
        dyn_id = DYN_BROADCAST_ID
        length = len(len(x) for x in parameters) + 4
        inst = DYN_INST_SYNC_WRITE
        data_len = len(parameters[0])-1
        checksum = (~(dyn_id+length+inst+reg_addr+data_len+\
                    sum(sum(x) for x in parameters))+256) % 256
        data = [255, 255, dyn_id, length, inst, reg_addr]
        data.extend(parameters)
        data.append(checksum)
        array = bytes(data)
        self._uart.write(array)
        time.sleep(0.001)
        self.last_error = DYN_ERR_INVALID

    # -------------
    # API Functions
    # -------------

    def set_register(self, dyn_id, reg_addr, data):
        params = [data]
        self.write_data(dyn_id, reg_addr, params)

    def set_register_dual(self, dyn_id, reg_addr, data):
        data1 = data & 0xFF
        data2 = data >> 8
        params = [data1,data2]
        self.write_data(dyn_id, reg_addr, params)

    def get_register(self, dyn_id, reg_addr):
        packet = self.read_data(dyn_id, reg_addr, 1)
        return packet[5]

    def get_register_dual(self, dyn_id, reg_addr):
        packet = self.read_data(dyn_id, reg_addr, 2)
        return packet[5] | (packet[6] << 8)

    # Like get_data, but only returns parameters, not the whole packet
    def get_bytes(self, dyn_id, reg_addr, nbytes):
        packet = self.read_data(dyn_id, reg_addr, nbytes)
        return packet[5:(4+nbytes)]

    def set_speed(self, dyn_id, speed):
        self.set_register_dual(dyn_id, DYN_REG_MOVING_SPEED_L, speed)

    def set_position(self, dyn_id, pos):
        self.set_register_dual(dyn_id, DYN_REG_GOAL_POSITION_L, pos)

    def get_temp(self, dyn_id):
        return self.get_register(dyn_id, DYN_REG_PRESENT_TEMP)

    def get_error(self, dyn_id):
        return self.ping(dyn_id)

    def parse_error(self, error=0xFF):
        if error == 0xFF:
            error = self.last_error

        if error == DYN_ERR_NONE:
            print("No Errors Reported\n")
        elif error & DYN_ERR_VOLTAGE:
            print("Voltage Error\n")
        elif error & DYN_ERR_ANGLE:
            print("Angle Limit Error\n")
        elif error & DYN_ERR_OVERHEAT:
            print("Overheat Error\n")
        elif error & DYN_ERR_RANGE:
            print("Instruction Range Error\n")
        elif error & DYN_ERR_CHECKSUM:
            print("Bad Checksum Error\n")
        elif error & DYN_ERR_OVERLOAD:
            print("Over Load Limit Error\n")
        elif error & DYN_ERR_INST:
            print("Invalid Instruction Error\n")
        elif error & DYN_ERR_INVALID:
            print("No errors available at startup, or for reset or broadcast"\
                   " instructions\n")

