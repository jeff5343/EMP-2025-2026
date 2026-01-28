#ifndef __ROBOT_H_INCLUDED__
#define __ROBOT_H_INCLUDED__

#include <vector>
#include <string>
#include "subsystems/drivetrain.h"
#include "commands/pid_drive.h"
#include "commands/pure_pursuit.h"
#include "util/structs/pose.h"
#include "util/structs/path.h"
#include "util/trapezoid_profile.h"

enum ALLIANCE {
    RED_TOP,
    RED_BOT,
    BLUE_TOP,
    BLUE_BOT
};

class Robot
{
private:
    // subsystems
    Drivetrain drivetrain{};

    PurePursuit purePursuit{drivetrain};

    PidDrive pidDrive{
        drivetrain,
        // turning pid constants
        PidConstants{0.4, 0.0, 0.0},
        // turning pid setpoint tolerance
        0.01,
        // straight pid constants
        PidConstants{0.05, 0.0, 0.0},
        // straight pid setpoint tolerance
        0.05,
        // turning profile constraints
        TrapezoidProfile::Constraints{2 * M_PI, 4 * M_PI},
        // straight profile constraints
        TrapezoidProfile::Constraints{60, 120},
    };

    // for testing
    const Pose poseSetpoints[4] = {
        Pose{10.0, 0.0, 0.0},
        Pose{1.0, 1.0, M_PI / 2.0},
        Pose{0.0, 1.0, M_PI},
        Pose{0.0, 0.0, -M_PI / 2.0},
    };
    int poseSetpointsLength = 1;
    int poseSetpointIndex = 0;
    bool pathFollowingStarted = false;

    // for paths
    const std::string pathFileName = "path1.txt";
    std::vector<Path> paths = {};
    std::vector<bool> backwards = {false, true, false};
    int pathIndex = 0;

    bool isCalibrating = true;

public:
    /* called in pre_auton */
    void init(ALLIANCE alliance);

    /* called every 20ms in autonomous */
    void autonomousPeriodic(int currentPathIndex);

    /* follow path */
    void followPaths();

    /* called every 20ms in usercontrol */
    void usercontrolPeriodic();

    /* logging statements */
    void log();

    /* auto run 1 */
    void autonomousRun1();

};

#endif