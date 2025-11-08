#ifndef __PID_H_INCLUDED__
#define __PID_H_INCLUDED__

#include "pid_constants.h"
#include <cmath>

/* very crude implementation, can be a lot more sophisticated */
class Pid
{
private:
    /* CONSTANTS */
    PidConstants constants;
    // in seconds
    const double deltaTime;
    const double setpointTolerance;

    // do the endpoints wrap around
    bool continuous = false;

    double setpoint;
    double prevError;
    double totalError;

public:
    Pid(PidConstants constants, double setpointTolerance = 0.05, double deltaTime = 0.02)
        : constants(constants), deltaTime(deltaTime), setpointTolerance(setpointTolerance) {};

    void enableContinuousInput(bool continuous)
    {
        this->continuous = continuous;
    }

    bool isContinuousInputEnabled()
    {
        return continuous;
    }

    void setSetpoint(double setpoint)
    {
        reset();
        this->setpoint = setpoint;
    }

    double getSetpoint()
    {
        return setpoint;
    }

    double getError()
    {
        return prevError;
    }

    bool isAtSetpoint()
    {
        return std::abs(prevError) < setpointTolerance;
    }

    double calculate(double measurement)
    {
        double error = setpoint - measurement;

        // choose shorter path for continuous
        // TODO: use modulus operator instead
        if (continuous && std::abs(error) > M_PI)
        {
            double convSetpoint, convMeasurement;
            if (setpoint > M_PI)
            {
                convSetpoint = setpoint - M_PI * 2;
                convMeasurement = measurement;
            }
            else
            {
                convSetpoint = setpoint;
                convMeasurement = measurement - M_PI * 2;
            }
            error = convSetpoint - convMeasurement;
        }

        totalError += error * deltaTime;
        double errorDerivative = (error - prevError) / deltaTime;

        prevError = error;

        return (constants.kP * error) + (constants.kI * totalError) + (constants.kD * errorDerivative);
    }

    void reset()
    {
        setpoint = 0;
        prevError = 0;
        totalError = 0;
    }
};

#endif