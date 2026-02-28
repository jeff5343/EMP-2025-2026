#ifndef __ROBOT_CONFIG_H_INCLUDED__
#define __ROBOT_CONFIG_H_INCLUDED__

#include "vex.h"
/**
 * All your components have to be initalized in one file
 * with the brain. I wasted so much time rip.
 */

inline extern vex::brain Brain{};

inline const vex::controller controller{};

// encoders
inline extern vex::rotation rightEncoder{vex::PORT9, false};
inline extern vex::rotation backEncoder{vex::PORT10, true};

// motors (top motors are the elevate ones)
constexpr bool LEFT_MOTORS_INVERTED = true;
constexpr bool RIGHT_MOTORS_INVERTED = false;

// LEFT MOTORS
inline extern vex::motor botFrontLeftMotor{
    vex::PORT5, vex::ratio6_1, LEFT_MOTORS_INVERTED};
inline extern vex::motor botBackLeftMotor{
    vex::PORT6, vex::ratio6_1, LEFT_MOTORS_INVERTED};
inline extern vex::motor topFrontLeftMotor{
    vex::PORT7, vex::ratio6_1, !LEFT_MOTORS_INVERTED};
inline extern vex::motor topBackLeftMotor{
    vex::PORT8, vex::ratio6_1, !LEFT_MOTORS_INVERTED};

// RIGHT MOTORS
inline extern vex::motor botFrontRightMotor{
    vex::PORT1, vex::ratio6_1, RIGHT_MOTORS_INVERTED};
inline extern vex::motor botBackRightMotor{
    vex::PORT2, vex::ratio6_1, RIGHT_MOTORS_INVERTED};
inline extern vex::motor topFrontRightMotor{
    vex::PORT3, vex::ratio6_1, !RIGHT_MOTORS_INVERTED};
inline extern vex::motor topBackRightMotor{
    vex::PORT4, vex::ratio6_1, !RIGHT_MOTORS_INVERTED};

// motor groups
inline extern vex::motor_group leftMotorGroup{botFrontLeftMotor, botBackLeftMotor, topFrontLeftMotor, topBackLeftMotor};
inline extern vex::motor_group rightMotorGroup{botFrontRightMotor, botBackRightMotor, topFrontRightMotor, topBackRightMotor};

// intake motors
inline extern vex::motor intakeMotor{
    vex::PORT11, vex::ratio6_1, false};

//throughtake motor
inline extern vex::motor throughtakeMotor{
    vex::PORT12, vex::ratio6_1, false}; // change port as needed

//outtake motors
inline extern vex::motor outakeMotor{
    vex::PORT13, vex::ratio6_1, false}; //change port as needed

// pistons
inline extern vex::digital_out trianglePiston = vex::digital_out(Brain.ThreeWirePort.H);
inline extern vex::digital_out outtakeElevationPiston = vex::digital_out(Brain.ThreeWirePort.C);
inline extern vex::digital_out intakeChutePiston = vex::digital_out(Brain.ThreeWirePort.A);
inline extern vex::digital_out descorePiston = vex::digital_out(Brain.ThreeWirePort.G);

// IMU
inline extern vex::inertial inertial{vex::PORT19};

#endif