#ifndef __ROBOT_H_INCLUDED__
#define __ROBOT_H_INCLUDED__

#include <vector>
#include <string>
#include "subsystems/drivetrain.h"
#include "subsystems/intake_outake.h"
#include "commands/pid_drive.h"
#include "commands/pure_pursuit.h"
#include "commands/heading_controller.h"
#include "util/structs/pose.h"
#include "util/structs/path.h"
#include "util/trapezoid_profile.h"
#include "util/alliance.h"

class Robot
{
private:
    // subsystems
    Drivetrain drivetrain{};
    IntakeOuttake intakeOuttake{};

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

    HeadingController headingController{
        drivetrain,
        // turning pid constants
        PidConstants{0.3, 0.0, 0.000},
        // kS
        0.02,
        // turning pid setpoint tolerance (~1.5 deg)
        0.026,
        // turning profile constraints
        TrapezoidProfile::Constraints{2 * M_PI, 4 * M_PI},
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
    vex::timer timer = vex::timer();

    // for paths
    const std::string autoPathFileName = "path1.txt";
    const std::string skillsPathFileName = "Skillsauto.txt";
    std::vector<Path> paths = {};
    std::vector<bool> backwards = {false, true, false, true, false, true};
    std::vector<bool> backwardsSkills = {false, true, false, false, 
                                         false, false, true, false, false, 
                                         false, false, true, false, false};
    int pathIndex = 0;

    bool isCalibrating = true;
    bool hasToggledIntakeChutePiston = false;
    bool hasToggledDescorePiston = false;


    static void toggleOuttakeHigh()
    {
        // I think we should check outtakeSpeed instead since this is the "variable speed"
        // in case user is coming from intake or can just check all three

        if (intakeSpeed <= 0 || throughtakeSpeed <= 0 || outtakeSpeed <= 0)
        {
            intakeSpeed = 0.8;
            throughtakeSpeed = 0.8;
            outtakeSpeed = 0.8;
        }
        else
        {
            intakeSpeed = 0;
            throughtakeSpeed = 0;
            outtakeSpeed = 0;
        }
    }
    static void toggleOuttakeMid()
    {
        if (intakeSpeed<=0 || throughtakeSpeed <=0 || outtakeSpeed >=0)
        {
            intakeSpeed =0.8;
            throughtakeSpeed = 0.8;
            outtakeSpeed = -0.8;
        }
        else
        {
            intakeSpeed =0;
            throughtakeSpeed =0;
            outtakeSpeed =0;
        }
    }
    static void toggleIntake()
    {
        if (intakeSpeed<=0 || throughtakeSpeed !=0 || outtakeSpeed !=0)
        {
            intakeSpeed = 0.8;
            throughtakeSpeed = 0;
            outtakeSpeed =0;
            //not doing throughtake to prevent balls from flying out since it would get stuck in mid score
        }
        else
        {
            intakeSpeed =0;
            throughtakeSpeed =0;
            outtakeSpeed =0;
        }
    }
    static void toggleReverseIntake()
    {
        //if intake speed is either stopped or currently intaking, switch everything to reverse
        if (intakeSpeed>=0|| throughtakeSpeed>=0 || outtakeSpeed>=0)
        {
            intakeSpeed =-0.8;
            throughtakeSpeed =-0.8;
            outtakeSpeed=-0.8;
        }
        //otherwise set everything to 0
        else
        {
            intakeSpeed =0;
            throughtakeSpeed =0;
            outtakeSpeed =0;
        }
    }

public:
    static double intakeSpeed;
    static double throughtakeSpeed;
    static double outtakeSpeed;   

    const bool IS_SKILLS = false;

    /* called in pre_auton */
    void init(ALLIANCE alliance);

    /* follow path */
    void followPaths();

    /* called every 20ms in usercontrol */
    void usercontrolPeriodic();

    /* logging statements */
    void log();

    void extendTriangle()
    {
        intakeOuttake.trianglePistonOut();
    }

    /* auto run 1 */
    void autonomousRun1();

    void skillz();

    void followPathCommand(int currentPathIndex, bool turning);

    void logStatements()
    {
        purePursuit.logStatements();
    }
    void goForwardSlowly(double speed);
    void backup(double speed);
    void autonomousIntake();
    void autonomousScoreLongGoal();
    void autonomousScoreLowGoal();
    void autonomousPark();
};

#endif