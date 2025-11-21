#ifndef __OUTTAKE_H_INCLUDED__
#define __OUTTAKE_H_INCLUDED__

#include "vex.h"
#include "odometry.h"

class OutTake
{

public:
    void intake();
    void eject();
    void stop();

private:
    void setPercent(double out);
    const int MAX_RPM = 600;
    const int REVERSE_SPEED = -0.7;
    const int INTAKE_SPEED = 0.7;
};

#endif