/*
Odometer.h
Source: https://github.com/DrGFreeman/RasPiBot202.V2

MIT License

Copyright (c) 2017 Julien de la Bruere-Terreault <drgfreeman@tuta.io>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
An odometer class for a differential drive robot
*/

#ifndef Odometer_h
#define Odometer_h

#include <Arduino.h>

class Odometer_h
{
public:
  /* Constructor
    tickDistNum & tickDistDenom: the numerator and denominator of a fraction
    defining the distance travelled for one encoder tick count. The numerator
    should be small enough to avoid overflow of a signed integer representing
    the distance travelled between two calls to the update method (distance =
    encoderCounts * numerator / denominator).
    track: the width between the wheels in number of tickDist
  */
  Odometer(unsigned int tickDistNum, unsigned int tickDistDenom,
    unsigned int track);

  // Return the left speed in dist_unit / s
  int getSpeedLeft();

  // Return the right speed in dist_unit / s
  int getSpeedRight();

  // Return the time step between last two update calls
  unsigned int getTimeStep();

  // Update odometer status
  void update(int countLeft, int countRight);

private:
  // Encoder counts attributes
  int _lastCountLeft;
  int _lastCoundRight;

  // Geomtrical attributes
  unsigned int _tickDistNum;
  unsigned int _tickDistDenom;
  unsigned int _track;

  // Time step attributes
  unsigned long _lastUpdateTime;
  unsigned int _timeStep;
};

#endif
