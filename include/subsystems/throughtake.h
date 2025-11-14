#ifndef __THROUGHTAKE_H_INCLUDED__
#define __THROUGHTAKE_H_INCLUDED__

#include "vex.h"
#include "odometry.h"

class ThroughTake
{

public:
    void intake();
    void reverse();

private:
    void setPercent(double out);
    const int MAX_RPM = 600;
    const int REVERSE_SPEED = -0.7;
    const int INTAKE_SPEED = 0.7;
};

#endif