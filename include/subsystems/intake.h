#ifndef __INTAKE_H_INCLUDED__
#define __INTAKE_H_INCLUDED__

#include "vex.h"
#include "odometry.h"

class InTake
{

public:
    void intake();
    void reverse();
    void clamp();
    void unclamp();
    void stop();

private:
    void setPercent(double out);
    const int MAX_RPM = 600;
    const int REVERSE_SPEED = -0.7;
    const int INTAKE_SPEED = 0.7;
};

#endif