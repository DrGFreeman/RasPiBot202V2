/*
AStarEncoders.cpp
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
An encoder class to read quadrature encoders
on the Pololu AStar 32U4 microcontroller.
*/

#include <AStarEncoders.h>

// Encoder pins
// !!! Update ISRs and constructor below if changing these pins !!!
// Left A: PCINT1, PB1, digital pin 15, PINB & 0B00000010 (0x02)
const byte LEFT_A = 15;
// Left B: PCINT2, PB2, digital pin 16, PINB & 0B00000100 (0x04)
const byte LEFT_B = 16;
// Right A: INT6, PE6, digital pin 7, PINE & 0B01000000 (0x40)
const byte RIGHT_A = 7;
// Right B: INT2, PD2, digital pin 0, PIND & 0B00000100 (0x04)
const byte RIGHT_B = 0;

// Last encoder readings
static volatile bool lastLeftA;
static volatile bool lastLeftB;
static volatile bool lastRightA;
static volatile bool lastRightB;

volatile uint16_t leftCount;
volatile uint16_t rightCount;

// ISRs
// ISR for left encoder pins A & B (PCINT1, PCINT2)
ISR(PCINT0_vect)
{
  bool newLeftA = PINB & 0x02; // Use direct access to PINB register bit 1
  bool newLeftB = PINB & 0x04; // Use direct access to PINB register bit 2

  leftCount += (newLeftA ^ lastLeftB) - (lastLeftA ^ newLeftB);

  lastLeftA = newLeftA;
  lastLeftB = newLeftB;
}

//ISR for right encoder pins A & B (INT6, INT2)
static void rightISR()
{
  bool newRightA = PINE & 0x40; // Use direct access to PINE register bit 6
  bool newRightB = PIND & 0x04; // Use direct access to PIND register bit 2

  rightCount += (newRightA ^ lastRightB) - (lastRightA ^ newRightB);

  lastRightA = newRightA;
  lastRightB = newRightB;
}

// Constructor
AStarEncoders::AStarEncoders()
{
  // Set encoder pins mode
  pinMode(LEFT_A, INPUT_PULLUP);
  pinMode(LEFT_B, INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT_PULLUP);
  pinMode(RIGHT_B, INPUT_PULLUP);

  // Enable PCINT1 & PCINT2; set 1 in PCMSK0 register bits 1 & 2
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);

  // Enable pin change interrupts; set 1 in PCICR register bit 1 (PCIE0)
  PCICR |= (1 << PCIE0);

  // Clear pin change interrupt flag; set 1 in PCIFR register bit 1 (PCIFR)
  PCIFR |= (1 << PCIF0);

  // Attach interrupts for right encoder
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_B), rightISR, CHANGE);

  // Initialize encoders variables
  lastLeftA = PINB & 0x02; // Use direct access to PINB register bit 4
  lastLeftB = PINB & 0x04; // Use direct access to PINB register bit 2
  lastRightA = PINE & 0x40; // Use direct access to PINE register bit 6
  lastRightB = PIND & 0x04; // Use direct access to PIND register bit 2
  leftCount = 0;
  rightCount = 0;

  flipDirection(false, false);
}

// Flip count directions
void AStarEncoders::flipDirection(bool left, bool right)
{
  if (left)
  {
    // Flip left encoder directions
    _flipLeft = true;
  }
  else
  {
    _flipLeft = false;
  }
  if (right)
  {
    // Flip right encoder directions
    _flipRight = true;
  }
  else
  {
    _flipRight = false;
  }
}

// Get left counts
int AStarEncoders::getCountsLeft()
{
  cli();
  int counts = leftCount;
  sei();

  if (_flipLeft)
  {
    counts *= -1;
  }

  return counts;
}

// Get right counts
int AStarEncoders::getCountsRight()
{
  cli();
  int counts = rightCount;
  sei();

  if (_flipRight)
  {
    counts *= -1;
  }

  return counts;
}

// Get left counts and reset left counts
int AStarEncoders::getCountsLeftAndReset()
{
  cli();
  int counts = leftCount;
  leftCount = 0;
  sei();

  if (_flipLeft)
  {
    counts *= -1;
  }

  return counts;
}

// Get right counts and reset right counts
int AStarEncoders::getCountsRightAndReset()
{
  cli();
  int counts = rightCount;
  rightCount = 0;
  sei();

  if (_flipRight)
  {
    counts *= -1;
  }

  return counts;
}
