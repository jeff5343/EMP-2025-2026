#ifndef __ROBOT_H_INCLUDED__
#define __ROBOT_H_INCLUDED__

#include "subsystems/drivetrain.h"
#include "commands/pid_drive.h"
#include "util/pose.h"
#include "util/trapezoid_profile.h"

class Robot
{
private:
    // subsystems
    Drivetrain drivetrain{};

    PidDrive pidDrive{
        drivetrain,
        // turning pid constants
        PidConstants{0.4, 0.0, 0.0},
        // straight pid constants
        PidConstants{0.05, 0.0, 0.0},
        // turning profile constraints
        TrapezoidProfile::Constraints{2 * M_PI, 4 * M_PI},
        // straight profile constraints
        TrapezoidProfile::Constraints{60, 120},
    };

    const vex::controller controller{};

    // for testing
    const Pose poseSetpoints[4] = {
        Pose{10.0, 0.0, 0.0},
        Pose{1.0, 1.0, M_PI / 2.0},
        Pose{0.0, 1.0, M_PI},
        Pose{0.0, 0.0, -M_PI / 2.0},
    };
    int poseSetpointsLength = 1;
    int poseSetpointIndex = 0;
    bool isPoseSetpointSet = false;

    bool isCalibrating = true;

public:
    /* called in pre_auton */
    void init();

    /* called every 20ms in autonomous */
    void autonomousPeriodic();

    /* called every 20ms in usercontrol */
    void usercontrolPeriodic();

    /* logging statements */
    void log();
};

#endif