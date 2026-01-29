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
    else if (controller.ButtonUp.pressing())
    {
        // point up
        if (!pathFollowingStarted)
        {
            pathFollowingStarted = true;
            headingController.setTargetHeading(0);
        }
        headingController.update();
    }
    else if (controller.ButtonDown.pressing())
    {
        // point down
        if (!pathFollowingStarted)
        {
            pathFollowingStarted = true;
            headingController.setTargetHeading(M_PI);
        }
        headingController.update();
    }
    else
    {
        // so that setTargetPose is ran when b is pressed ahfwahf
        pathFollowingStarted = false;

        // convert axis positions to range -1.0 to 1.0
        double x = static_cast<double>(controller.Axis1.position()) / 100.0;
        double y = static_cast<double>(controller.Axis3.position()) / 100.0;

        // printf("ux: %.3f, uy: %.3f", x, y);

        // apply deadband
        double deadband = 0.03;
        x = MathUtil::axisPower(MathUtil::deadband(x, deadband), 1.3);
        y = MathUtil::axisPower(MathUtil::deadband(y, deadband), 1.3);

        double staticFriction = 0.03;
        if (std::fabs(x) > 0)
            x += std::copysign(staticFriction, x);
        if (std::fabs(y) > 0)
            y += std::copysign(staticFriction, y);

        // printf("x: %.3f, y: %.3f\n", x, y);

        drivetrain.arcadeDrive(MathUtil::deadband(x, deadband), MathUtil::deadband(y, deadband));
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