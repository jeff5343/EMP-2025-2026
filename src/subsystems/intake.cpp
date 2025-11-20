#include "subsystems/intake.h"

void InTake::setPercent(double out)
{
    intakeLeftMotor.spin(vex::forward, (out * MAX_RPM), vex::rpm);
    intakeRightMotor.spin(vex::forward, (out * MAX_RPM), vex::rpm);
}

void InTake::intake()
{
    setPercent(INTAKE_SPEED);
}

void InTake::reverse()
{
    setPercent(REVERSE_SPEED);
}

void InTake::clamp()
{
    mouth.set(true);
}

void InTake::unclamp()
{
    mouth.set(false);
}

void InTake::stop()
{
    setPercent(0);
}