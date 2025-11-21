#include "subsystems/throughtake.h"

void ThroughTake::setPercent(double out)
{
    throughTakeMotor.spin(vex::forward, (out * MAX_RPM), vex::rpm);
}

void ThroughTake::intake()
{
    setPercent(INTAKE_SPEED);
}

void ThroughTake::reverse()
{
    setPercent(REVERSE_SPEED);
}

void ThroughTake::stop()
{
    setPercent(0);
}