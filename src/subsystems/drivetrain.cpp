#include "subsystems/drivetrain.h"

#include <cmath>
#include <algorithm>

Drivetrain::Drivetrain()
{
    if (IS_BRAKE_MODE)
    {
        leftMotorGroup.setStopping(vex::brake);
        rightMotorGroup.setStopping(vex::brake);
    }
    else
    {
        leftMotorGroup.setStopping(vex::coast);
        rightMotorGroup.setStopping(vex::coast);
    }
}

void Drivetrain::setVelocityLeftMotors(double rpm)
{
    leftMotorGroup.spin(vex::forward, rpm, vex::rpm);
}

void Drivetrain::setVelocityRightMotors(double rpm)
{
    rightMotorGroup.spin(vex::forward, rpm, vex::rpm);
}

void Drivetrain::stop()
{
    leftMotorGroup.stop();
    rightMotorGroup.stop();
}

void Drivetrain::arcadeDrive(double x, double y)
{
    double max = std::max(std::fabs(x), std::fabs(y));
    double total = y + x, difference = y - x;

    double leftOut, rightOut;

    if (x >= 0 && y >= 0)
    {
        // Q1
        leftOut = max;
        rightOut = difference;
    }
    else if (x <= 0 && y >= 0)
    {
        // Q2
        leftOut = total;
        rightOut = max;
    }
    else if (x <= 0 && y <= 0)
    {
        // Q3
        leftOut = -max;
        rightOut = difference;
    }
    else
    {
        // Q4
        leftOut = total;
        rightOut = -max;
    }

    setVelocityLeftMotors(leftOut * Drivetrain::MAX_RPM);
    setVelocityRightMotors(rightOut * Drivetrain::MAX_RPM);
}

void Drivetrain::log()
{
    odometry.getPose().print();
}