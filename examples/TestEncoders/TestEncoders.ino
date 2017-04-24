/*
TestEncoders.ino
Source: https://github.com/DrGFreeman/RasPiBot202V2

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
Test encoders
*/

#include <AStar32U4.h>
#include <AStarEncoders.h>

AStarEncoders encoders;
AStar32U4Motors motors;
AStar32U4ButtonA btnA;
AStar32U4ButtonB btnB;

void setup() {
  Serial.begin(9600);
  encoders.flipDirection(false, true);
}

void loop() {
  int left = encoders.getCountsLeft();
  int right = encoders.getCountsRight();
  Serial.print(left);
  Serial.print("\t");
  Serial.println(right);
  if (btnA.isPressed())
  {
    motors.setSpeeds(100, 100);
  }
  else if (btnB.isPressed())
  {
    motors.setSpeeds(-100, -100);
  }
  else
  {
    motors.setSpeeds(0, 0);
  }
  delay(25);
}
