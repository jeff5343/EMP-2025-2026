#ifndef __TRAPEZOID_PROFILE_H_INCLUDED__
#define __TRAPEZOID_PROFILE_H_INCLUDED__

#include <cmath>

/**
 * direct copy from FRC:
 * https://frcdocs.wpi.edu/en/2020/docs/software/advanced-control/controllers/trapezoidal-profiles.html
 * */

class TrapezoidProfile
{
public:
    struct Constraints
    {
        double maxVelocity;
        double maxAcceleration;
    };
    struct State
    {
        double position;
        double velocity;
    };

    TrapezoidProfile(Constraints constraints)
        : constraints(constraints), goalState({0, 0}) {};
    TrapezoidProfile(Constraints constraints, State initialState, State goalState)
        : constraints(constraints), goalState(goalState) {};

    /* returns position at time in the trapezoid profile */
    State calculate(double time, State current)
    {
        // limit max velocity
        if (std::fabs(current.velocity) > constraints.maxVelocity)
            current.velocity = sgn(current.velocity) * constraints.maxVelocity;

        // acceleration direction
        int acc_direction = current.position < goalState.position ? 1 : -1;
        // flip positions/velocities based on direction
        // for arithmetic operations, will be reverted
        State currentS = current;
        currentS.position *= acc_direction;
        currentS.velocity *= acc_direction;
        State goalS = goalState;
        goalS.position *= acc_direction;
        goalS.velocity *= acc_direction;

        // account for negaive/positive inital or final velocity
        double cutoffBegin_sec = currentS.velocity / constraints.maxAcceleration;
        double cutoffBegin_dist = 0.5 * constraints.maxAcceleration * cutoffBegin_sec * cutoffBegin_sec;

        double cutoffEnd_sec = goalS.velocity / constraints.maxAcceleration;
        double cutoffEnd_dist = 0.5 * constraints.maxAcceleration * cutoffEnd_sec * cutoffEnd_sec;

        double fullTrapezoid_dist = cutoffBegin_dist + (goalS.position - currentS.position) + cutoffEnd_dist;
        double accTime_sec = constraints.maxVelocity / constraints.maxAcceleration;

        double fullSpeed_dist = fullTrapezoid_dist - accTime_sec * accTime_sec * constraints.maxAcceleration;

        // max velocity is never reached (triangle)
        if (fullSpeed_dist < 0)
        {
            accTime_sec = sqrt(fullTrapezoid_dist / constraints.maxAcceleration);
            fullSpeed_dist = 0;
        }

        double endAcc_sec = accTime_sec - cutoffBegin_sec;
        double endFullSpeed_sec = endAcc_sec + fullSpeed_dist / constraints.maxVelocity;
        double endDecel_sec = endFullSpeed_sec + accTime_sec - cutoffEnd_sec;

        State result = currentS;

        if (time < endAcc_sec)
        {
            result.velocity += time * constraints.maxAcceleration;
            result.position += (currentS.velocity + 0.5 * constraints.maxAcceleration * time) * time;
        }
        else if (time < endFullSpeed_sec)
        {
            result.velocity = constraints.maxVelocity;
            result.position +=
                (currentS.velocity + 0.5 * constraints.maxAcceleration * endAcc_sec) * endAcc_sec +
                +((time - endAcc_sec) * constraints.maxVelocity);
        }
        else if (time <= endDecel_sec)
        {
            result.velocity = goalS.velocity + (endDecel_sec - time) * constraints.maxAcceleration;
            double timeLeft = endDecel_sec - time;
            result.position = goalS.position -
                              (goalS.velocity + timeLeft * constraints.maxAcceleration / 2.0) * timeLeft;
        }
        else
        {
            result = goalS;
        }

        result.position *= acc_direction;
        result.velocity *= acc_direction;
        return result;
    }

    State getGoalState() { return goalState; }
    void setGoalState(State goalState)
    {
        this->goalState = goalState;
    }

private:
    Constraints constraints;
    State goalState;

    // TODO: put into helper class?
    template <typename T>
    int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }
};

#endif