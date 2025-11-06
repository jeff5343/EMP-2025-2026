#ifndef __PID_DRIVE_H_INCLUDED__
#define __PID_DRIVE_H_INCLUDED__

#include "util/pid_constants.h"
#include "util/profiled_pid.h"
#include "subsystems/drivetrain.h"
#include "vex.h"

class PidDrive
{
private:
    ProfiledPid headingController;
    ProfiledPid straightController;
    Pose target;

    Drivetrain &drivetrain;

    double startingTargetAngle;
    bool hasReachedAngle = false;

    double calculateTargetAngle(Pose current);
    double calculateErrorDist(Pose current);

public:
    PidDrive(Drivetrain &drivetrain, PidConstants headingPidConstants, PidConstants straightPidConstants,
             TrapezoidProfile::Constraints headingProfileConstraints,
             TrapezoidProfile::Constraints straightProfileConstraints)
        : drivetrain(drivetrain), headingController(headingPidConstants, headingProfileConstraints, 0.01),
          straightController(straightPidConstants, straightProfileConstraints)
    {
        headingController.enableContinuousInput(true);
    };

    void setTargetPose(Pose pose);
    void update();
    bool isAtTargetPose();
};

#endif