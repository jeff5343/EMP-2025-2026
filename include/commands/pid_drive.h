#ifndef __PID_DRIVE_H_INCLUDED__
#define __PID_DRIVE_H_INCLUDED__

#include "util/pid_constants.h"
#include "util/pid.h"
#include "subsystems/drivetrain.h"

class PidDrive
{
private:
    Pid anglePid;
    Pid straightPid;
    Pose target;

    Drivetrain &drivetrain;

    double startingTargetAngle;

public:
    PidDrive(PidConstants anglePidConstants, PidConstants straightPidConstants, Drivetrain &drivetrain)
        : anglePid(anglePidConstants), straightPid(straightPidConstants), drivetrain(drivetrain)
    {
        anglePid.enableContinuousInput(true);
    };

    void setTargetPose(Pose pose);
    void update();
    bool isAtSetpoint();
};

#endif