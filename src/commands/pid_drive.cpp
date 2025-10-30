#include "commands/pid_drive.h"
#include "util/math_util.h"
#include "util/angle.h"

void PidDrive::setTargetPose(Pose pose)
{
    target = pose;
    straightPid.setSetpoint(0);
    anglePid.setSetpoint(pose.radians);
}

// TODO: put into helper
template <typename T>
T signum(T x)
{
    return (x > 0) - (x < 0);
}

void PidDrive::update()
{
    Pose current = drivetrain.getPose();

    // calculate error to target pose
    double errorX = target.x - current.x;
    double errorY = target.y - current.y;
    double errorDist = sqrt(errorX * errorX + errorY * errorY) * signum<double>(errorX); // signum is a cheap fix
    double errorAngle = atan2(errorY, errorX);                                           // TODO: i don think this is accurate?
    // i think it gives your desired angle?
    // TODO: fix this logic ahhh

    // PID based on error to target (setpoints are 0)
    // TODO: clamp pid values
    double straightPidOut = -MathUtil::clamp(straightPid.calculate(errorDist), -.1, .1);
    double turnPidOut = MathUtil::clamp(anglePid.calculate(current.radians), -.1, .1);

    // output values to left and right wheels
    double leftOut = 0, rightOut = 0;

    // moving to pose
    if (!straightPid.isAtSetpoint())
    {
        // first point to pose
        if (!anglePid.isAtSetpoint())
        {
            leftOut = -turnPidOut;
            rightOut = turnPidOut;
        }
        // drive to pose
        else
        {
            leftOut = straightPidOut;
            rightOut = straightPidOut;
        }
    }

    printf("angle error: %.3f\n", Angle::toDegrees(errorAngle));
    printf("dist error: %.3f\n", errorDist);
    printf("leftOut: %.3f\n", leftOut);
    printf("calcOutput: %.3f\n", anglePid.calculate(errorAngle));

    drivetrain.setPercentOut(leftOut, rightOut);

    // TODO: add rotating to pose defined angle at the end!
    // if (anglePid.isAtSetpoint()) {
    //     anglePid.setSetpoint(target.radians);
    // }
}

bool PidDrive::isAtSetpoint()
{
    return anglePid.isAtSetpoint() && straightPid.isAtSetpoint();
}