#include "commands/pid_drive.h"
#include "util/math_util.h"
#include "util/angle.h"

void PidDrive::setTargetPose(Pose pose)
{
    target = pose;

    Pose current = drivetrain.getPose();
    // calculate starting target angle
    startingTargetAngle = calculateTargetAngle(current);
    hasReachedAngle = false;

    straightController.setGoal(0);
    headingController.setGoal(startingTargetAngle);
    headingController.reset(current.radians);
}

double PidDrive::calculateTargetAngle(Pose current)
{
    double errorX = target.x - current.x;
    double errorY = target.y - current.y;
    double targetAngle = atan2(errorY, errorX);
    if (targetAngle < 0)
        targetAngle += M_PI * 2.0;
    return targetAngle;
}

double PidDrive::calculateErrorDist(Pose current)
{
    double errorX = target.x - current.x;
    double errorY = target.y - current.y;
    return sqrt(errorX * errorX + errorY * errorY);
}

void PidDrive::update()
{
    Pose current = drivetrain.getPose();
    double targetAngle = calculateTargetAngle(current);
    double errorDist = calculateErrorDist(current);

    headingController.setGoal(targetAngle);

    // cheap fix, but if the target angle changes over 135
    // from the original target angle then robot probably
    // went over the goal and we can reverse direction
    // (maybe cross product??)
    double angleDifference = 180 - std::fabs(std::fabs(targetAngle - startingTargetAngle) - 180);
    if (angleDifference > 3.0 * M_PI / 4.0)
    {
        printf("NEGATING!!!!\n");
        errorDist *= -1;
    }

    // output values to left and right wheels
    double leftOut = 0, rightOut = 0;

    // moving to pose
    if (!straightController.isAtGoal())
    {
        // first point to pose
        if (!headingController.isAtGoal() && errorDist > 0)
        {
            printf("angling!!!\n");
            double turnOut = headingController.calculate(targetAngle);
            leftOut = -turnOut;
            rightOut = turnOut;
            hasReachedAngle = false;
        }
        // drive to pose
        else
        {
            printf("straighting!!!\n");
            if (!hasReachedAngle)
            {
                straightController.reset(errorDist);
                hasReachedAngle = true;
            }
            double straightOut = straightController.calculate(errorDist);
            leftOut = straightOut;
            rightOut = straightOut;
        }
    }

    drivetrain.setPercentOut(leftOut, rightOut);

    // printf("target angle: %.3f\n", Angle::toDegrees(targetAngle));
    // printf("starting target angle: %.3f\n", Angle::toDegrees(startingTargetAngle));
    // printf("current angle angle: %.3f\n", Angle::toDegrees(current.radians));
    // // printf("angle error: %.3f\n", Angle::toDegrees(errorAngle));
    // // printf("dist error: %.3f\n", errorDist);
    // printf("leftOut: %.3f\n", leftOut);

    // TODO: add rotating to pose defined angle at the end!
    // if (headingController.isAtSetpoint()) {
    //     headingController.setSetpoint(target.radians);
    // }
}

bool PidDrive::isAtTargetPose()
{
    return headingController.isAtGoal() && straightController.isAtGoal();
}
