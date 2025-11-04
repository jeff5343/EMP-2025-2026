#ifndef __PID_DRIVE_H_INCLUDED__
#define __PID_DRIVE_H_INCLUDED__

#include "util/pid_constants.h"
#include "util/pid.h"
#include "util/trapezoid_profile.h"
#include "subsystems/drivetrain.h"
#include "vex.h"

class PidDrive
{
private:
    Pid anglePid;
    Pid straightPid;
    Pose target;
    // TODO: make a ProfiledPID to make it easier to use trap profiles
    TrapezoidProfile angleTrapProfile;
    TrapezoidProfile straightTrapProfile;
    vex::timer straightProfileTimer{};

    bool angleReached = false;

    Drivetrain &drivetrain;

    double startingTargetAngle;

public:
    PidDrive(Drivetrain &drivetrain, PidConstants anglePidConstants, PidConstants straightPidConstants,
             TrapezoidProfile::Constraints angleProfileConstraints,
             TrapezoidProfile::Constraints straightProfileConstraints)
        : drivetrain(drivetrain), anglePid(anglePidConstants), straightPid(straightPidConstants),
          angleTrapProfile(angleProfileConstraints), straightTrapProfile(straightProfileConstraints)
    {
        anglePid.enableContinuousInput(true);
    };

    void setTargetPose(Pose pose);
    void update();
    bool isAtSetpoint();
};

#endif