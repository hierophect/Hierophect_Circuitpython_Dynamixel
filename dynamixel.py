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
import time

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

DYN_INST_PING                   = const(0x01)
DYN_INST_READ                   = const(0x02)
DYN_INST_WRITE                  = const(0x03)
DYN_INST_REG_WRITE              = const(0x04)
DYN_INST_ACTION                 = const(0x05)
DYN_INST_RESET                  = const(0x06)
DYN_SYNC_WRITE                  = const(0x83)

class Dynamixel:
    def __init__(self, uart, direction_pin):
        self._uart = uart
        self._dir = direction_pin
        # what do do here? Report all connected motors? 

    def set_register(self, dyn_id, reg_addr, data):
        length = 4
        inst = DYN_INST_WRITE
        data = data & 0xFF
        checksum = (~(dyn_id+length+inst+reg_addr+data)+256) % 256
        data = [255, 255, dyn_id, length, inst, reg_addr, data, checksum]
        array = bytes(data)
        self._dir.value = True
        self._uart.write(array)

    def set_register_dual(self, dyn_id, reg_addr, data):
        length = 5
        inst = DYN_INST_WRITE
        data1 = data & 0xFF
        data2 = data >> 8
        checksum = (~(dyn_id+length+inst+reg_addr+data1+data2)+256) % 256
        data = [255, 255, dyn_id, length, inst, reg_addr, data1, data2, checksum]
        array = bytes(data)
        self._dir.value = True
        self._uart.write(array)

    def write_data(self, dyn_id, reg_addr, parameters):
        length = len(parameters) + 3
        inst = DYN_INST_WRITE
        checksum = (~(dyn_id+length+inst+reg_addr+sum(parameters))+256) % 256
        data = [255, 255, dyn_id, length, inst, reg_addr]
        data.extend(parameters)
        data.append(checksum)
        array = bytes(data)
        self._dir.value = True
        self._uart.write(array)

    def read_data(self, dyn_id, reg_addr, nbytes):
        length = 4
        inst = DYN_INST_READ
        checksum = (~(dyn_id+length+inst+reg_addr+nbytes)+256) % 256
        data = [255,255,dyn_id,length,inst,reg_addr,nbytes,checksum]
        array = bytes(data)
        self._uart.write(array)
        time.sleep(0.001)
        self._dir.value = False
        retval = self._uart.read(nbytes+6)
        self._dir.value = True
        return retval

    def get_register(self, dyn_id, reg_addr):
        retval = self.read_data(dyn_id, reg_addr, 1)
        if retval[4] != 0:
            print("Error" + str(int(retval[4])));
        return retval[5]

    # TODO
    # def write_reg(self, dyn_id, reg_addr, data):
    # def write_reg_dual(self, dyn_id, reg_addr, data):

    def set_speed(self, dyn_id, speed):
        self.set_register_dual(dyn_id, DYN_REG_MOVING_SPEED_L, speed)

    def set_pos(self, dyn_id, pos):
        self.set_register_dual(dyn_id, DYN_REG_GOAL_POSITION_L, pos)

    def get_temp(self, dyn_id):
        self.get_register(dyn_id, DYN_REG_PRESENT_TEMP)


