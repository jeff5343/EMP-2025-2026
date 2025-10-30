#include "robot.h"
#include <cmath>

void Robot::init()
{
    /* CALIBRATE INERTIAL SENSOR */
    isCalibrating = true;
    inertial.calibrate();
    while (inertial.installed() && inertial.isCalibrating())
    {
        brain.Screen.clearScreen();
        brain.Screen.print("Calibrating inertial sensor...");
        vex::this_thread::sleep_for(50);
    }
    inertial.resetHeading();
    drivetrain.startOdometry();
    isCalibrating = false;
};

void Robot::usercontrolPeriodic()
{
    /* don't run anything until inertial is calibrated */
    // i dont think this is needed, while loop should block thread
    if (isCalibrating)
    {
        printf("calibrating...");
        return;
    }

    /* TELEOP DRIVING: */

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

    // TODO: find out how to bind functions to events??
    if (controller.ButtonA.pressing())
    {
        drivetrain.resetOdometry(0, 0, 0);
    }

    /* ODOMETRY TESTING: */
    drivetrain.log();

    brain.Screen.clearLine();

    brain.Screen.print("x: ");
    brain.Screen.print(drivetrain.getPose().x);
    brain.Screen.print(", y: ");
    brain.Screen.print(drivetrain.getPose().y);
    brain.Screen.print(", deg: ");
    brain.Screen.print(drivetrain.getPose().radians * (180 / M_PI));
}