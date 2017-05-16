# motioncontroller.py
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
# This module defines the MotionController class which offers methods to
# control movement of a differential drive robot. The MotionController class
# interfaces with the AStar class.

import time

# Constants definition

ACCEL_FIXED, ACCEL_NONE = range(2)

# MotionController class definition

class MotionController:
    def __init__(self, aStar):
        """MotionController class constructor."""
        # AStar interface object
        self._aStar = aStar
        # Set default forward acceleration to 200 mm/s^2
        self._fwdAccel = 200.
        # Set forward acceleration mode to ACCEL_AUTO
        self._fwdAccelMode = 0#ACCEL_FIXED
        # Set default maximum forward speed to 400 mm/s
        self._fwdSpeedMax = 400.
        # Initialize current forward speed to 0
        self._fwdSpeedCurrent = 0.
        # Initialize _lastUpdateTime to current time
        self._lastUpdateTime = time.time()
        # Initialize _timeStep to 0
        self._timeStep = 0.01

    def setFwdAccel(self, fwdAccel):
        """Sets the fixed forward acceleration in mm/s to use with accel mode
        ACCEL_FIXED."""
        self._fwdAccel = fwdAccel

    def setFwdAccelMode(self, accelMode):
        """Sets the acceleration mode"""
        self._fwdAccelMode = accelMode

    def setSpeeds(self, fwdSpeed, turnRate):
        """Sets the robot speed in mm/s and turn rate in radians/s."""
        self._updateTimeStep()
        # Get speed and turn rate considering accelerations
        fwdSpeed = self._getFwdSpeedAccelDecel(fwdSpeed)
        # Check speed and turn rate do not exceed maximum values
        fwdSpeed = _constrain(fwdSpeed, -self._fwdSpeedMax, self._fwdSpeedMax)
        # Set AStar speeds
        self.aStar.setSpeeds(fwdSpeed, turnRate)

    def stop(self):
        """Stops the robot immediately."""
        # Set AStar speeds to 0
        self.aStar.setSpeeds()

    # Private methods

    def _constrain(value, minVal, maxVal):
        """Constrain a input value to be within a min and max range"""
        if value < minVal:
            value = minVal
        elif value > maxVal:
            value = maxVal
        return value

    def _getFwdSpeedAccel(self, speedTarget):
        """Returns the fwd speed considering acceleration."""
        if self._fwdAccelMode == ACCEL_FIXED:
            # Check direction of speed change
            if speedTarget - self._fwdSpeedCurrent >= 0:
                deltaDir = 1
            else:
                deltaDir = -1
            # Calculate new speed based on fixed acceleration
            newSpeed = self._fwdSpeedCurrent + deltaDir * \
                self._fwdAccel * self._timeStep
            # Check that new speed does not exceed target speed
            if abs(newSpeed) > abs(speedTarget):
                newSpeed = speedTarget
        else:
            # No acceleration
            newSpeed = speedTarget

        # Update current speed
        self._fwdSpeedCurrent = newSpeed
        return newSpeed

    def _updateTimeStep(self):
        """Update time step."""
        t = time.time()
        self._timeStep = t - self._lastUpdateTime
        self._lastUpdateTime = t
