#ifndef __ROBOT_H_INCLUDED__
#define __ROBOT_H_INCLUDED__

#include "subsystems/drivetrain.h"
#include "commands/pid_drive.h"
#include "util/pose.h"

class Robot
{
private:
    // subsystems
    Drivetrain drivetrain{};

    PidDrive pidDrive{
        // turning pid constants
        PidConstants{1.0, 0.0, 0.0}, 
        // straight pid constants
        PidConstants{1.0, 0.0, 0.0}, 
        drivetrain};

    const vex::controller controller{};

    // for testing
    const Pose poseSetpoints[4] = {
        Pose{1.0, 0.0, 0.0},
        Pose{1.0, 1.0, M_PI / 2.0},
        Pose{0.0, 1.0, M_PI},
        Pose{0.0, 0.0, -M_PI / 2.0},
    };
    int poseSetpointsLength = 4;
    int poseSetpointIndex = 0;
    bool isPoseSetpointSet = false;

public:
    /* called in pre_auton */
    void init();

    /* called every 20ms in autonomous */
    void autonomousPeriodic();

    /* called every 20ms in usercontrol */
    void usercontrolPeriodic();
};

#endif