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
    straightController.reset(calculateErrorDist(current));
    headingController.setGoal(startingTargetAngle);
    headingController.reset(current.radians);
    headingController.calculate(current.radians);
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

    double angleDifference = M_PI - std::fabs(std::fabs(targetAngle - (current.radians)) - M_PI);
    // HEHEHE I THINK I FIGURE IT OUT!!! ONG UYAY IF THIS DOESNT
    // WORK I WILL DO SOMETHING!!! AND NOT WIGJ DAND jwnfKAFWn
    double dotErrorDist = cos(angleDifference) * errorDist;

    // cheap fix, but if the target angle changes over 135
    // from the original target angle then robot probably
    // went over the goal and we can reverse direction
    // (maybe cross product??)
    // if (angleDifference > 3.0 * M_PI / 4.0)
    // {
    //     printf("NEGATING!!!!\n");
    //     // targetAngle -= M_PI;
    //     errorDist *= -1;
    // }

    headingController.setGoal(targetAngle);

    // output values to left and right wheels
    double leftOut = 0, rightOut = 0;

    // moving to pose
    if (!straightController.isAtGoal())
    {
        // first point to pose
        if (!headingController.isAtGoal() && std::fabs(errorDist) && !hasReachedAngle)
        {
            printf("angling!!!\n");
            if (hasReachedAngle)
            {
                headingController.reset(current.radians);
                hasReachedAngle = false;
            }
            printf("setpoint: %.3f, goal: %.3f\n", headingController.getSetpoint().position, headingController.getGoalState().position);
            printf("current: %.3f\n", current.radians);
            double turnOut = headingController.calculate(current.radians);
            leftOut = -turnOut;
            rightOut = turnOut;
        }
        // drive to pose
        else
        {
            printf("straighting!!!\n");
            if (!hasReachedAngle)
            {
                straightController.reset(dotErrorDist);
                hasReachedAngle = true;
            }
            printf("setpoint: %.3f, goal: %.3f\n", straightController.getSetpoint().position, straightController.getGoalState().position);
            double straightOut = -straightController.calculate(dotErrorDist);
            leftOut = straightOut;
            rightOut = straightOut;
        }
    }

    printf("heading error: %.3f\n", headingController.getError());
    printf("out: %.3f\n", leftOut);
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
