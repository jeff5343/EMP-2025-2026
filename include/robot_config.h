#ifndef __ROBOT_CONFIG_H_INCLUDED__
#define __ROBOT_CONFIG_H_INCLUDED__

#include "vex.h"

/**
 * I didn't know all your components had to be initalized in one file
 * with the brain. I wasted so much time rip.
 */

inline extern vex::brain brain{};

inline const vex::controller controller{};

// encoders
inline extern vex::rotation rightEncoder{vex::PORT10, false};
inline extern vex::rotation backEncoder{vex::PORT16, false};

// motors (top motors are the elevate ones)
constexpr bool LEFT_MOTORS_INVERTED = true;
constexpr bool RIGHT_MOTORS_INVERTED = false;
inline extern vex::motor frontLeftMotor{
    vex::PORT4, vex::ratio6_1, LEFT_MOTORS_INVERTED};
inline extern vex::motor topLeftMotor{
    vex::PORT6, vex::ratio6_1, !LEFT_MOTORS_INVERTED};
inline extern vex::motor backLeftMotor{
    vex::PORT5, vex::ratio6_1, LEFT_MOTORS_INVERTED};
inline extern vex::motor frontRightMotor{
    vex::PORT1, vex::ratio6_1, RIGHT_MOTORS_INVERTED};
inline extern vex::motor topRightMotor{
    vex::PORT3, vex::ratio6_1, !RIGHT_MOTORS_INVERTED};
inline extern vex::motor backRightMotor{
    vex::PORT2, vex::ratio6_1, RIGHT_MOTORS_INVERTED};

// motor groups
inline extern vex::motor_group leftMotorGroup{frontLeftMotor, topLeftMotor, backLeftMotor};
inline extern vex::motor_group rightMotorGroup{frontRightMotor, topRightMotor, backRightMotor};

// IMU
inline extern vex::inertial inertial{vex::PORT19}; // TODO: set correct port

#endif