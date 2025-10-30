#include "commands/pid_drive.h"
#include "util/math_util.h"
#include "util/angle.h"

void PidDrive::setTargetPose(Pose pose)
{
    target = pose;
    straightPid.setSetpoint(0);

    Pose current = drivetrain.getPose();

    // calculate starting target angle
    double errorX = target.x - current.x;
    double errorY = target.y - current.y;
    double targetAngle = atan2(errorY, errorX);
    if (targetAngle < 0)
        targetAngle += M_PI * 2.0;
    startingTargetAngle = targetAngle;
    anglePid.setSetpoint(targetAngle);
}

void PidDrive::update()
{
    Pose current = drivetrain.getPose();

    // calculate error to target pose
    double errorX = target.x - current.x;
    double errorY = target.y - current.y;

    double targetAngle = atan2(errorY, errorX);
    if (targetAngle < 0)
        targetAngle += M_PI * 2.0;
    double errorAngle = targetAngle - current.radians;
    anglePid.setSetpoint(targetAngle);

    double errorDist = sqrt(errorX * errorX + errorY * errorY);
    // cheap fix, but if the target angle changes over 135
    // from the original target angle then robot probably
    // went over the goal and we can reverse direction
    double angleDifference = 180 - std::fabs(std::fabs(targetAngle - startingTargetAngle) - 180);
    if (angleDifference > 3.0 * M_PI / 4.0)
    {
        printf("NEGATING!!!!");
        errorDist *= -1;
    }

    // PID based on error to target (setpoints are 0)
    double straightPidOut = -MathUtil::clamp(straightPid.calculate(errorDist), -.2, .2);
    double turnPidOut = MathUtil::clamp(anglePid.calculate(current.radians), -.5, .5);

    // output values to left and right wheels
    double leftOut = 0, rightOut = 0;

    // moving to pose
    if (!straightPid.isAtSetpoint())
    {
        // first point to pose
        if (!anglePid.isAtSetpoint() && errorDist > 0)
        {
            printf("angling!!!");
            leftOut = -turnPidOut;
            rightOut = turnPidOut;
        }
        // drive to pose
        else
        {
            printf("straighting!!!");
            leftOut = straightPidOut;
            rightOut = straightPidOut;
        }
    }

    drivetrain.setPercentOut(leftOut, rightOut);

    printf("target angle: %.3f\n", Angle::toDegrees(targetAngle));
    printf("starting target angle: %.3f\n", Angle::toDegrees(startingTargetAngle));
    printf("current angle angle: %.3f\n", Angle::toDegrees(current.radians));
    // printf("angle error: %.3f\n", Angle::toDegrees(errorAngle));
    // printf("dist error: %.3f\n", errorDist);
    printf("leftOut: %.3f\n", leftOut);
    printf("calcOutput: %.3f\n", anglePid.calculate(errorAngle));

    // TODO: add rotating to pose defined angle at the end!
    // if (anglePid.isAtSetpoint()) {
    //     anglePid.setSetpoint(target.radians);
    // }
}

bool PidDrive::isAtSetpoint()
{
    return anglePid.isAtSetpoint() && straightPid.isAtSetpoint();
}