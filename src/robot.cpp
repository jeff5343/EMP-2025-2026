#include "robot.h"
#include "util/angle.h"
#include "util/path_parser.h"
#include <cmath>

void Robot::init()
{
    /* CALIBRATE INERTIAL SENSOR */
    isCalibrating = true;
    inertial.calibrate();
    while (inertial.installed() && inertial.isCalibrating())
    {
        brain.Screen.clearLine();
        brain.Screen.print("Calibrating inertial sensor...");
        printf("calibrating...\n");
        vex::this_thread::sleep_for(100);
    }
    inertial.resetHeading();
    drivetrain.startOdometry();
    isCalibrating = false;
    printf("done calibrating!\n");



    paths = PathParser::loadPaths(pathFileName);
    double startX = paths[0].points[0][0];
    double startY = paths[0].points[0][1];

    drivetrain.resetOdometry(startX, startY, M_PI); //need to get starting points of path not just the first path we need to follow


    purePursuit.setPath(paths[0].points, true);
    printf("done loading %lu paths!\n", paths.size());
};

void Robot::usercontrolPeriodic()
{
    /* DON'T RUN IF CALIBRATING */
    if (isCalibrating)
        return;

    /* TELEOP DRIVING: */
    if (controller.ButtonR1.pressing())
    {
        intakeRightMotor.spin(vex::forward, 10, vex::voltageUnits::volt);
        intakeLeftMotor.spin(vex::forward, 10, vex::voltageUnits::volt);
    }
    else if (controller.ButtonL1.pressing())
    {
        intakeRightMotor.spin(vex::forward, -10, vex::voltageUnits::volt);
        intakeLeftMotor.spin(vex::forward, -10, vex::voltageUnits::volt);
    }
    else
    {
        intakeRightMotor.spin(vex::forward, 0, vex::voltageUnits::volt);
        intakeLeftMotor.spin(vex::forward, 0, vex::voltageUnits::volt);
    }

    // TODO: find out how to bind functions to events??
    if (controller.ButtonA.pressing())
    {
        drivetrain.resetOdometry(-47.2795, 46.5748, M_PI);
    }

    if (controller.ButtonB.pressing())
    {
        if (!pathFollowingStarted)
        {
            pidDrive.setTargetPose(poseSetpoints[poseSetpointIndex]);
            pathFollowingStarted = true;
        }
        pidDrive.update();
    }
    else if (controller.ButtonY.pressing())
    {
        if (!pathFollowingStarted)
        {
            pathFollowingStarted = true;
            pathIndex = 0;
            if (paths.size() > 0)
            {
                purePursuit.setPath(paths[pathIndex].points, backwards[pathIndex]);
            }
            purePursuit.reset();
        }
        followPaths();
    }
    else
    {
        // so that setTargetPose is ran when b is pressed ahfwahf
        pathFollowingStarted = false;

        // convert axis positions to range -1.0 to 1.0
        double x = static_cast<double>(controller.Axis1.position()) / 100.0;
        double y = static_cast<double>(controller.Axis3.position()) / 100.0;

        double deadband = 0.05;
        if (std::fabs(x) <= deadband)
            x = 0;
        if (std::fabs(y) <= deadband)
            y = 0;
        if (std::fabs(x) > deadband || std::fabs(y) > deadband)
            drivetrain.arcadeDrive(x, y);
        else
            drivetrain.stop();
    }

    /* logging */
    log();
}

void Robot::followPaths()
{
    if (pathIndex >= paths.size())
    {
        drivetrain.setPercentOut(0, 0);
        return;
    }

    purePursuit.update();

    if (purePursuit.isAtGoal())
    {
        printf("path: %d complete\n", pathIndex);
        pathIndex++;
        if (pathIndex >= paths.size())
        {
            drivetrain.setPercentOut(0, 0);
            return;
        }
        purePursuit.setPath(paths[pathIndex].points, backwards[pathIndex]);
    }
}

void Robot::autonomousPeriodic()
{
    if (!pathFollowingStarted)
    {
        pathFollowingStarted = true;
        pathIndex = 0;
        if (paths.size() > 0)
        {
            purePursuit.setPath(paths[0].points, backwards[pathIndex]);
        }
        purePursuit.reset();
    }
    purePursuit.update();
}

void Robot::log()
{
    /* subsystem logging */
    // drivetrain.log();

    /* brain logging */
    brain.Screen.clearLine();

    // brain.Screen.print("rightD: ");
    // brain.Screen.print(drivetrain.getOdometry().getRightDist());
    brain.Screen.print(", heading: ");
    brain.Screen.print(Angle::toDegrees(drivetrain.getPose().radians));
    brain.Screen.print(", x: ");
    brain.Screen.print(drivetrain.getPose().x);
    brain.Screen.print(", y: ");
    brain.Screen.print(drivetrain.getPose().y);
    // brain.Screen.print(", total deg: ");
    // brain.Screen.print(Angle::toDegrees(drivetrain.getOdometry().getTotalRadians()));
}