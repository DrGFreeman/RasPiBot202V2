/*
AStarRPiSlave.ino
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
AStar Raspberry Pi Slave
This is the main sketch to run on the A-Star 32U4 Robot Controller for the
RPB202 robot.
*/

#include <AStar32U4.h>
#include <PololuRPiSlave.h>
#include <AStarEncoders.h>
#include <Odometer.h>
#include <TimedPID.h>
#include <PololuMaestro.h>

// Define robot geometrical properties
// Distance travelled per encoder tick
const float tickDist = .152505;
// Width between the two wheels
const float track = 142.5;

// Define motor PID gains
const float Kp = 0.4;
const float Ki = 1.9;
const float Kd = 0.0;

// Define motors max command
const float motorsMaxCommand = 400;

// Define motor trims in 1/40
const int leftTrim = 40;
const int rightTrim = 40;

// Define speed variables for acceleration control
int lastSpeedCmdLeft = 0;
int lastSpeedCmdRight = 0;

// Define maximum speed command change per time step
const int accelMax = 10;

// Define different objects from RasPiBot202V2 library
// Encoders
AStarEncoders encoders;
// Odometer
Odometer odometer(tickDist, track);
// Motor PID controllers
TimedPID pidLeft(Kp, Ki, Kd);
TimedPID pidRight(Kp, Ki, Kd);

// Define objects from AStar32U4 library
// Buzzer
PololuBuzzer buzzer;
// Motors
AStar32U4Motors motors;
// Buttons
AStar32U4ButtonA btnA;
AStar32U4ButtonB btnB;
AStar32U4ButtonC btnC;

// Define servo channels
const byte panServoCh = 0;
const byte tiltServoCh = 1;
const byte mastServoCh = 2;

// Define Pololu Mini Maestro object & serial port
#define maestroSerial SERIAL_PORT_HARDWARE_OPEN
MicroMaestro maestro(maestroSerial);

// Data exchanged with the Raspberry Pi
struct Data
{
  bool yellow, green, red;
  bool buttonA, buttonB, buttonC;

  int16_t fwdSpeed, turnRate;
  int16_t x, y, phi;

  bool resetOdometer;

  uint16_t batteryMillivolts;

  bool playNotes;
  char notes[14];

  uint16_t panServo, tiltServo, mastServo;
};

// Define Pololu RPi Slave objects
PololuRPiSlave<struct Data,5> slave;

void setup() {
  // Set up the slave at I2C address 20
  slave.init(20);

  // Start serial communication with the Mini Maestro
  maestroSerial.begin(9600);

  // Play startup sound
  buzzer.play("V10>>g16>>>c16");
  delay(200);

  // Set encoders directions
  encoders.flipDirection(false, true);

  // Set PID controllers command range
  pidLeft.setCmdRange(-motorsMaxCommand, motorsMaxCommand);
  pidRight.setCmdRange(-motorsMaxCommand, motorsMaxCommand);

  if (btnA.isPressed())
  {
    // Center servos
    maestro.setTarget(panServoCh, 6000);
    maestro.setTarget(tiltServoCh, 6000);
    maestro.setTarget(mastServoCh, 6000);
    delay(500);
    // Release servos
    maestro.setTarget(panServoCh, 0);
    maestro.setTarget(tiltServoCh, 0);
    maestro.setTarget(mastServoCh, 0);
  }
}

void loop() {
  // Get current time
  unsigned long currentTime = micros();

  // Read encoder counts
  int countsLeft = encoders.getCountsLeft();
  int countsRight = encoders.getCountsRight();

  // Update odometer
  odometer.update(countsLeft, countsRight);

  /* Call updateBuffer() before using the buffer, to get the latest
  data including recent master writes. */
  slave.updateBuffer();

  // Write button states into data structure
  slave.buffer.buttonA = btnA.isPressed();
  slave.buffer.buttonB = btnB.isPressed();
  slave.buffer.buttonC = btnC.isPressed();

  // Write battery level into data structure
  slave.buffer.batteryMillivolts = readBatteryMillivoltsSV();

  // Read reset odometer flag and reset odometer values if true
  if (slave.buffer.resetOdometer)
  {
    odometer.reset();
  }

  // Write odometry values into data structure
  slave.buffer.x = int(odometer.getX());
  slave.buffer.y = int(odometer.getY());
  slave.buffer.phi = int(odometer.getPhi() * 1000);

  // Read LED values from buffer and set LED states
  ledYellow(slave.buffer.yellow);
  ledGreen(slave.buffer.green);
  ledRed(slave.buffer.red);

  // Read servo positions from buffer and write to servo objects
  maestro.setTarget(panServoCh, slave.buffer.panServo);
  maestro.setTarget(tiltServoCh, slave.buffer.tiltServo);
  maestro.setTarget(mastServoCh, slave.buffer.mastServo);

  // Read speed and turn rate from buffer and calculate motor speeds
  float fwdSpeed = float(slave.buffer.fwdSpeed);
  float turnRate = float(slave.buffer.turnRate) / 1000;
  float leftSpeed = fwdSpeed - turnRate * track / 2;
  float rightSpeed = fwdSpeed + turnRate * track / 2;

  // Set motor speeds
  setMotorSpeeds(leftSpeed, rightSpeed);

  // Playing music involves both reading and writing, since we only
  // want to do it once.
  static bool startedPlaying = false;

  if(slave.buffer.playNotes && !startedPlaying)
  {
    buzzer.play(slave.buffer.notes);
    startedPlaying = true;
  }
  else if (startedPlaying && !buzzer.isPlaying())
  {
    slave.buffer.playNotes = false;
    startedPlaying = false;
  }

  // We are done writing; make modified data available to I2C master
  slave.finalizeWrites();

  // Ensure a constant time step of the main loop (10 milliseconds)
  while (micros() - currentTime < 10000)
  {
    // Do nothing; wait until time step is reached
  }
}

// Sets the motor speeds using PID controllers
void setMotorSpeeds(int speedLeft, int speedRight)
{
  // get speed command from PID controllers
  int speedCmdLeft = pidLeft.getCmdAutoStep(speedLeft, odometer.getSpeedLeft());
  int speedCmdRight = pidRight.getCmdAutoStep(speedRight, odometer.getSpeedRight());

  // Handle speed commands

  // Control maximum acceleration
  if (speedCmdLeft - lastSpeedCmdLeft > accelMax)
  {
    speedCmdLeft = lastSpeedCmdLeft + accelMax;
  }
  if (speedCmdLeft - lastSpeedCmdLeft < -accelMax)
  {
    speedCmdLeft = lastSpeedCmdLeft - accelMax;
  }
  if (speedCmdRight - lastSpeedCmdRight > accelMax)
  {
    speedCmdRight = lastSpeedCmdRight + accelMax;
  }
  if (speedCmdRight - lastSpeedCmdRight < -accelMax)
  {
    speedCmdRight = lastSpeedCmdRight - accelMax;
  }

  // Stop immediately if target speed is zero
  if (speedLeft == 0)
  {
    speedCmdLeft = 0;
    pidLeft.reset();
  }
  if (speedRight == 0)
  {
    speedCmdRight = 0;
    pidRight.reset();
  }

  // Set motor speeds
  motors.setSpeeds(speedCmdLeft * leftTrim / 40, speedCmdRight * rightTrim / 40);

  lastSpeedCmdLeft = speedCmdLeft;
  lastSpeedCmdRight = speedCmdRight;
}
