#include "subsystems/outtake.h"

void OutTake::setPercent(double out)
{
    outTakeMotor.spin(vex::forward, (out * MAX_RPM), vex::rpm);
}
void OutTake::intake()
{
    setPercent(INTAKE_SPEED);
}
void OutTake::eject()
{
    setPercent(REVERSE_SPEED);
}
void OutTake::stop()
{
    setPercent(0);
}