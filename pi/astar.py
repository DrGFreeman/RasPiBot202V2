# astar.py
# Source: https://github.com/DrGFreeman/RasPiBot202V2
#
# MIT License
#
# Copyright (c) 2017 Julien de la Bruere-Terreault <drgfreeman@tuta.io>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# This module defines the AStar class as a python interface to the Pololu
# A-Star 32U4 robot controller with Raspberry Pi bridge.

import time
import threading
import smbus
import struct
import sys

class AStar:
    def __init__(self):
        self._bus = smbus.SMBus(1)
        self._version = sys.version_info.major
        self.ledYellow = 0
        self.ledGreen = 0
        self.ledRed = 0
        self._buttonA = 0
        self._buttonB = 0
        self._buttonC = 0
        self._fwdSpeed = 0
        self._turnRate = 0
        self._lockSpeeds = False
        self._x = 0
        self._y = 0
        self._phi = 0
        self._lockOdometer = False
        self._batteryMV = 0
        self._lockBattery = False
        self._panServo = 0      # Servo is disabled by default
        self._tiltServo = 0     # Servo is disabled by default
        self._mastServo = 0     # Servo is disabled by default
        self._lockServos = False
        self._notes = ''
        self._resetOdometer = True
        self.run()
        # Wait to ensure we can read/write the buffer once before starting
        time.sleep(.05)
        # Print battery level
        print("RPB202")
        print("Battery level: " + str(round(self.getBatteryVolts(), 2)) + "V")

    def _read_unpack(self, address, size, format):
        """Reads data from the I2C bus."""
        self._bus.write_byte(20, address)
        time.sleep(0.0001)
        byte_list = [self._bus.read_byte(20) for _ in range(size)]
        if self._version == 3:
            # Python version 3
            return struct.unpack(format, bytes(byte_list))
        else:
            # Python version 2
            return struct.unpack(format, bytes(bytearray(byte_list)))

    def _write_pack(self, address, format, *data):
        """Writes data to the I2C bus."""
        if self._version == 3:
            # Python version 3
            data_array = list(struct.pack(format, *data))
        else:
            # Python version 2
            data_array = map(ord, list(struct.pack(format, *data)))
        self._bus.write_i2c_block_data(20, address, data_array)
        time.sleep(0.0001)

    def close(self):
        """Stops the I2C communication with the A-Star controller. This method
        also stops the motors and turns off the A-Star LEDs."""
        # Stop the running thread
        self._active = False
        # Stop the motors
        self.setSpeeds(0, 0)
        # Write the motors speeds directly to the I2C bus
        self._write_pack(6, 'hh', 0, 0)
        # Turn LEDs off
        self.setYellowLED(0)
        self.setGreenLED(0)
        self.setRedLED(0)
        # Write the LED values directly to the I2C bus
        self._write_pack(0, 'BBB', 0, 0, 0)

    def run(self):
        """Starts continuous I2C communication with A-Star controller in a
        dedicated thread."""
        self._active = True
        th = threading.Thread(target = self._run, args = [])
        th.start()

    def _run(self):
        """Runs continuous I2C communication with A-Star controller. Runs as
        long as AStar._active attribute is True. Call AStar.close() to stop the
        thread."""
        while self._active:
            try:
                # Read from buffer

                # Buttons
                self._buttonA, self._buttonB, self._buttonC = \
                    self._read_unpack(3, 3, '???')

                # Odometer
                self._lockOdometer = True
                self._x, self._y, phi = self._read_unpack(10, 6, 'hhh')
                # Convert phi reading from 1/1000 of radians to radians
                self._phi = phi / 1000.
                self._lockOdometer = False

                # Battery level
                self._lockBattery = True
                self._batteryMV = self._read_unpack(17, 2, 'H')[0]
                self._lockBattery = False

                # Write to buffer

                # Reset odometer on start-up
                if self._resetOdometer:
                    self._resetOdometer = False
                    self._write_pack(16, 'B', 1)
                    time.sleep(.02)
                else:
                    self._write_pack(16, 'B', 0)

                # LEDs
                self._write_pack(0, 'BBB', self.ledYellow, self.ledGreen, \
                    self.ledRed)

                # Servos
                self._lockServos = True
                self._write_pack(34, 'HHH', self._panServo, self._tiltServo, \
                    self._mastServo)
                self._lockServos = False

                # Notes
                if self._notes != "":
                    self._write_pack(19, 'B15s', 1, self._notes.encode('ascii'))
                    self._notes = ""

                # Motors (turn rate in 1/1000 of radians / s)
                self._lockSpeeds = True
                turnRate = int(self._turnRate * 1000)
                self._write_pack(6, 'hh', self._fwdSpeed, turnRate)
                self._lockSpeeds = False

            except IOError:
                # Handle I2C communication error
                raise IOError("IOError in AStar class")
                self.close()

    def buttonAIsPressed(self):
        """Returns True if the A-Star button A is pressed, False otherwise."""
        return self._buttonA

    def buttonBIsPressed(self):
        """Returns True if the A-Star button B is pressed, False otherwise."""
        return self._buttonB

    def buttonCIsPressed(self):
        """Returns True if the A-Star button C is pressed, False otherwise."""
        return self._buttonC

    def getBatteryVolts(self):
        """Returns the robot battery level in Volts."""
        while self._lockBattery:
            # Wait while battery attribute is locked
            pass
        return self._batteryMV / 1000.

    def getOdometerPhi(self):
        """Returns the phi angle of the robot from the odometer in radians
        (0 <= phi < 2*Pi). 0 corresponds to the robot pointing in the positive x
        direction. The angle increases turning in direction of the positive y
        axis (left turn).
        """
        while self._lockOdometer:
            # Wait while odometer attributes are locked
            pass
        return self._phi

    def getOdometerXY(self):
        """Returns the x and y position of the robot from the odometer in mm."""
        while self._lockOdometer:
            # Wait while odometer attributes are locked
            pass
        return self._x, self._y

    def setYellowLED(self, value = 0):
        """Sets the A-Star yellow led status (0 = Off, 1 = On)."""
        if value == 0:
            self.ledYellow = 0
        else:
            self.ledYellow = 1

    def setGreenLED(self, value = 0):
        """Sets the A-Star green led status (0 = Off, 1 = On)."""
        if value == 0:
            self.ledGreen = 0
        else:
            self.ledGreen = 1

    def setRedLED(self, value = 0):
        """Sets the A-Star red led status (0 = Off, 1 = On)."""
        if value == 0:
            self.ledRed = 0
        else:
            self.ledRed = 1

    def setPanServo(self, us_4 = 0):
        """Sets the pan servo pulse width value in quarter-microseconds."""
        while self._lockServos:
            # Wait while servos attributes are locked
            pass
        self._panServo = us_4

    def setTiltServo(self, us_4 = 0):
        """Sets the tilt servo pulse width value in quarter-microseconds."""
        while self._lockServos:
            # Wait while servos attributes are locked
            pass
        self._tiltServo = us_4

    def setMastServo(self, us_4 = 0):
        """Sets the mast servo pulse width value in quarter-microseconds."""
        while self._lockServos:
            # Wait while servos attributes are locked
            pass
        self._mastServo = us_4

    def playNotes(self, notes):
        """Play the specified notes on the A-Star buzzer. Refer to the Pololu
        Buzzer documentation for details on how to use the buzzer."""
        self._notes = notes

    def resetOdometer(self):
        """Resets the odometer on the A-Star."""
        self._resetOdometer = True

    def setSpeeds(self, fwdSpeed = 0, turnRate = 0):
        """Sets the robot speed in mm/s and turn rate in radians/s"""
        while self._lockSpeeds:
            # Wait while speds attributes are locked
            pass
        self._fwdSpeed = fwdSpeed
        self._turnRate = turnRate
