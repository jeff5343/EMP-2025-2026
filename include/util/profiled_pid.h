#ifndef __PROFILED_PID_H__
#define __PROFILED_PID_H__

#include "trapezoid_profile.h"
#include "pid.h"
#include "math_util.h"

/**
 * from wpilib:
 * https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/controller/ProfiledPIDController.java#L233
 */

class ProfiledPid
{
public:
    ProfiledPid(PidConstants pidConstants, TrapezoidProfile::Constraints profileConstraints)
        : controller(pidConstants), profile(profileConstraints) {};

    void enableContinuousInput(bool continuous)
    {
        controller.enableContinuousInput(continuous);
    }

    void setGoal(double goalMeasurement)
    {
        profile.setGoalState(TrapezoidProfile::State{goalMeasurement, 0});
    }

    double calculate(double measurement)
    {
        TrapezoidProfile::State goal = profile.getGoalState();

        if (controller.isContinuousInputEnabled())
        {
            double errorBound = 2 * M_PI / 2.0;
            double goalMinDistance = MathUtil::inputModulus(
                goal.position - measurement, -errorBound, errorBound);
            double setpointMinDistance = MathUtil::inputModulus(
                setpoint.position - measurement, -errorBound, errorBound);

            goal.position = goalMinDistance + measurement;
            setpoint.position = setpointMinDistance + measurement;
        }

        profile.setGoalState(goal);
        setpoint = profile.calculate(0.02, setpoint);
        controller.setSetpoint(setpoint.position);
        return controller.calculate(measurement);
    }

    bool isAtGoal()
    {
        return isAtSetpoint() && setpoint.position == profile.getGoalState().position && setpoint.velocity == profile.getGoalState().velocity;
    }

    bool isAtSetpoint()
    {
        return controller.isAtSetpoint();
    }

    TrapezoidProfile::State getSetpoint()
    {
        return setpoint;
    }

    TrapezoidProfile::State getGoalState()
    {
        return profile.getGoalState();
    }

    void reset(double measurement)
    {
        controller.reset();
        setpoint = TrapezoidProfile::State{measurement, 0};
    }

private:
    Pid controller;
    TrapezoidProfile profile;
    TrapezoidProfile::State setpoint;
};

#endif