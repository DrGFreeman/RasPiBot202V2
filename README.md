# RasPiBot202V2
A simple differential drive robot project based on Pololu Romi chassis and A-Star 32U4 SV w Raspberry Pi bridge

By Julien de la Bruère-Terreault (drgfreeman@tuta.io)

![Image of RPB-202 robot](https://github.com/DrGFreeman/RasPiBot202V2/raw/master/doc/img/rpb202-2.640px.jpg)

## Summary
This library is a work in progress and is the second generation of my RasPiBot202 robot. The first generation library, [DrGFreeman/RasPiBot202](https://github.com/DrGFreeman/RasPiBot202), remains available for reference but will not be developed further.

This new library aims at improving the performance of the control system by transferring more of the low-level, timing sensitive duties such as odometry and motor PID controls from the Raspberry Pi to the A-Star 32U4 microcontroller.

## Dependencies

#### Arduino
The following Arduino libraries are required by this library:  
* [pololu/a-star-32u4-arduino-library](https://github.com/pololu/a-star-32u4-arduino-library)
* [pololu/pololu-rpi-slave-arduino-library](https://github.com/pololu/pololu-rpi-slave-arduino-library)
* [pololu/maestro-arduino](https://github.com/pololu/maestro-arduino)
* [DrGFreeman/TimedPID](https://github.com/drgfreeman/TimedPID)

#### Python
The following Python libraries are required by this library:

* [DrGFreeman/PyTools](https://github.com/DrGFreeman/PyTools) (timedpid module)
