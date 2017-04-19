# RasPiBot202.V2
A simple differential drive robot project based on Pololu Romi chassis and A-Star 32U4 SV w Raspberry Pi bridge

[![Build Status](https://travis-ci.org/DrGFreeman/RasPiBot202.V2.svg?branch=master)](https://travis-ci.org/DrGFreeman/RasPiBot202.V2)  
By Julien de la Bruère-Terreault (drgfreeman@tuta.io)

## Summary
This library is a work in progress and is the second generation of my RasPiBot202 robot. The first generation library, [DrGFreeman/RasPiBot202](https://github.com/DrGFreeman/RasPiBot202), remains available for reference but will not be developed further.

This new library aims at improving the performance of the control system by transferring more of the low-level, timing sensitive duties such as odometry and motor PID controls from the Raspberry Pi to the A-Star 32U4 microcontroller.

## Dependencies
The following libraries are used by this library:  
* [pololu/a-star-32u4-arduino-library](https://github.com/pololu/a-star-32u4-arduino-library)
