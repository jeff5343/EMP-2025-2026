#include <cstdio>

#include "mock_vex.h"
#include "../include/util/pid.h"
#include "../include/util/angle.h"

/** Command in order to run:
 * g++ --std=c++11 pid_tests.cpp -D__TESTING__ && ./a.out
 */

// If parameter is not true, test fails
#define IS_TRUE(x)                                                    \
    {                                                                 \
        if (!(x))                                                     \
            printf("%s FAILED on line %d\n", __FUNCTION__, __LINE__); \
        else                                                          \
            printf("%s passed on line %d\n", __FUNCTION__, __LINE__); \
    }

void simple(Pid &pid)
{
    pid.setSetpoint(1);
    double output = pid.calculate(0);
    IS_TRUE(output == 5);
}

void continuous(Pid &pid)
{
    // shortest path from 45 deg to 315 deg
    pid.enableContinuousInput(true);
    pid.setSetpoint((7.0 * M_PI) / 4.0);
    double output = pid.calculate(M_PI / 4.0);
    IS_TRUE(output < 0);

    // shortest path from 315 deg to 45 deg
    pid.setSetpoint(M_PI / 4.0);
    output = pid.calculate((7.0 * M_PI) / 4.0);
    IS_TRUE(output > 0);

    // shortest path from 45 deg to 90 deg
    pid.setSetpoint(M_PI / 2.0);
    output = pid.calculate(M_PI / 4.0);
    IS_TRUE(output > 0);

    // shortest path from 90 deg to 45 deg
    pid.setSetpoint(M_PI / 4.0);
    output = pid.calculate(M_PI / 2.0);
    IS_TRUE(output < 0);
}

void isAtSetpoint(Pid &pid)
{
    pid.enableContinuousInput(false);
    pid.setSetpoint(1.0);
    double current = 0.0;
    pid.calculate(0.0);
    while (!pid.isAtSetpoint())
    {
        current += pid.calculate(current) * 0.05;
        // printf("%.3f, ", current);
    }
    IS_TRUE(pid.isAtSetpoint());
}

int main()
{
    Pid pid{PidConstants{5, 0, 0}};

    /* SIMPLE OUTPUT CHECK */
    simple(pid);
    /* CONTINUOUS OUTPUT CHECK */
    continuous(pid);
    /* IS AT SETPOINT */
    isAtSetpoint(pid);

    return 0;
}