#include "robot.h"
#include "util/angle.h"
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
};

void Robot::usercontrolPeriodic()
{
    /* DON'T RUN IF CALIBRATING */
    if (isCalibrating)
        return;

    /* TELEOP DRIVING: */

    if (controller.ButtonR1.pressing())
    {
        superstructure.groundIntake();
    }
    else if (controller.ButtonR2.pressing())
    {
        superstructure.bottomScoringOut();
    }
    else if (controller.ButtonL1.pressing())
    {
        superstructure.armScoringOut();
    }
    else if (controller.ButtonL2.pressing())
    {
        if (hasBeenPressed == false and superstructure.isArmUp())
        {
            superstructure.raisedScoreingPosition();
            hasBeenPressed = true;
        }
        else if (hasBeenPressed == false and !superstructure.isArmUp())
        {
            superstructure.levelScoringPosition();
            hasBeenPressed = true;
        }
    }
    else
    {
        hasBeenPressed = false;
        superstructure.stopAllMotors();
    }

    // TODO: find out how to bind functions to events??
    if (controller.ButtonA.pressing())
    {
        drivetrain.resetOdometry(0, 0, 0);
    }

    if (controller.ButtonB.pressing())
    {
        if (!isPoseSetpointSet)
        {
            pidDrive.setTargetPose(poseSetpoints[poseSetpointIndex]);
            isPoseSetpointSet = true;
        }
        pidDrive.update();
    }
    else
    {
        // so that setTargetPose is ran when b is pressed ahfwahf
        isPoseSetpointSet = false;

        // convert axis positions to range -1.0 to 1.0
        double x = static_cast<double>(controller.Axis1.position()) / 100.0;
        double y = static_cast<double>(controller.Axis3.position()) / 100.0;

        double deadband = 0.01;
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

void Robot::autonomousPeriodic()
{
    if (!isPoseSetpointSet)
    {
        pidDrive.setTargetPose(poseSetpoints[poseSetpointIndex]);
        isPoseSetpointSet = true;
    }
    pidDrive.update();

    if (pidDrive.isAtTargetPose())
    {
        poseSetpointIndex += 1;
        if (poseSetpointIndex >= poseSetpointsLength)
        {
            poseSetpointIndex = 0;
        }
    }
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