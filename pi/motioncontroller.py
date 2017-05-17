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
from math import pi

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
        # Set forward acceleration mode to ACCEL_FIXED
        self._fwdAccelMode = ACCEL_FIXED
        # Set default maximum forward speed to 400 mm/s
        self._fwdSpeedMax = 400.
        # Initialize current forward speed to 0
        self._fwdSpeedCurrent = 0.
        # Set default turn acceleration to pi rad/s^2
        self._turnAccel = pi
        # Set turn acceleration mode to ACCEL_FIXED
        self._turnAccelMode = ACCEL_FIXED
        # Set default maximum turn rate to 2 * pi rad/s
        self._turnRateMax = 2 * pi
        # Initialize current turn rate to 0
        self._turnRateCurrent = 0.
        # Initialize _lastUpdateTime to current time
        self._lastUpdateTime = time.time()
        # Initialize _timeStep to .01 s
        self._timeStep = .01

    def getFwdSpeed(self):
        """Returns the current forward speed in mm/s."""
        return self._fwdSpeedCurrent

    def getSpeeds(self):
        """Returns the current forward speed and turn rate."""
        return self._fwdSpeedCurrent, self._turnRateCurrent

    def getTurnRate(self):
        """Returns the current turn rate in rad/s."""
        return self._turnRateCurrent

    def setFwdAccel(self, fwdAccel):
        """Sets the fixed forward acceleration in mm/s^2 to use with accel mode
        ACCEL_FIXED."""
        self._fwdAccel = fwdAccel

    def setFwdAccelMode(self, accelMode):
        """Sets the forward acceleration mode"""
        self._fwdAccelMode = accelMode

    def setTurnAccel(self, turnAccel):
        """Sets the fixed turn acceleration in rad/s^2 to use with accel mode
        ACCEL_FIXED."""
        self._turnAccel = turnAccel

    def setTurnAccelMode(self, accelMode):
        """Sets the turn acceleration mode"""
        self._turnAccelMode = accelMode

    def setSpeeds(self, fwdSpeed, turnRate):
        """Sets the robot speed in mm/s and turn rate in radians/s."""
        self._updateTimeStep()
        # Get speed and turn rate considering accelerations
        fwdSpeed = self._getFwdSpeedAccel(fwdSpeed)
        turnRate = self._getTurnRateAccel(turnRate)
        # Set AStar speeds
        self._aStar.setSpeeds(fwdSpeed, turnRate)

    def stop(self):
        """Stops the robot immediately."""
        # Set AStar speeds to 0
        self._aStar.setSpeeds()

    # Private methods

    def _constrain(self, value, minVal, maxVal):
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
            if (deltaDir == 1 and newSpeed > speedTarget) or \
                (deltaDir == -1 and newSpeed < speedTarget):
                newSpeed = speedTarget
        else:
            # No acceleration
            newSpeed = speedTarget

        # Check speed does not exceed maximum speed
        newSpeed = self._constrain(newSpeed, -self._fwdSpeedMax, \
                    self._fwdSpeedMax)

        # Update current speed
        self._fwdSpeedCurrent = newSpeed
        return newSpeed

    def _getTurnRateAccel(self, turnRateTarget):
        """Returns the turn rate considering acceleration."""
        if self._turnAccelMode == ACCEL_FIXED:
            # Check direction of turn rate change
            if turnRateTarget - self._turnRateCurrent >= 0:
                deltaDir = 1
            else:
                deltaDir = -1
            # Calculate new turn rate based on fixed acceleration
            newTurnRate = self._turnRateCurrent + deltaDir * \
                self._turnAccel * self._timeStep
            # Check that new turn rate does not exceed turn rate target
            if (deltaDir == 1 and newTurnRate > turnRateTarget) or \
                (deltaDir == -1 and newTurnRate < turnRateTarget):
                newTurnRate = turnRateTarget
        else:
            # No acceleration
            newTurnRate = turnRateTarget

        # Check turn rate does not exceed max turn rate
        newTurnRate = self._constrain(newTurnRate, -self._turnRateMax, \
                        self._turnRateMax)

        # Update current turn rate
        self._turnRateCurrent = newTurnRate
        return newTurnRate

    def _updateTimeStep(self):
        """Update time step."""
        t = time.time()
        # If stopped, use a pre-defined time step
        if self._fwdSpeedCurrent == 0 and self._turnRateCurrent == 0:
            self._timeStep = .01
        else:
            self._timeStep = t - self._lastUpdateTime
        self._lastUpdateTime = t
