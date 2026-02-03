#ifndef __HEADING_CONTROLLER_H_INCLUDED__
#define __HEADING_CONTROLLER_H_INCLUDED__

#include "util/structs/pid_constants.h"
#include "util/profiled_pid.h"
#include "subsystems/drivetrain.h"
#include "vex.h"

class HeadingController
{
private:
    ProfiledPid headingController;
    double targetRadians;
    double kS;

    Drivetrain &drivetrain;

public:
    HeadingController(Drivetrain &drivetrain, PidConstants headingPidConstants, double kS,
                      double headingSetpointTolerance, TrapezoidProfile::Constraints headingProfileConstraints)
        : drivetrain(drivetrain), headingController(headingPidConstants, headingProfileConstraints, headingSetpointTolerance)
    {
        headingController.enableContinuousInput(true);
        this->kS = kS;
    };

    void setTargetHeading(double radians);
    void update();
    bool isAtTarget();

    void goToTargetHeadingCommand(double radians);
};

#endif