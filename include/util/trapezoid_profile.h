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

    TrapezoidProfile(Constraints constraints, State initialState, State goalState)
        : constraints(constraints), initialState(initialState), goalState(goalState) {};

    /* returns position at time in the trapezoid profile */
    double calculate(double time)
    {
        // limit max velocity
        if (std::fabs(initalState.velocity) > constraints.maxVelocity)
            initialState.velocity = sgn(initS.velocity) * constraints.maxVelocity;

        // acceleration direction
        int acc_direction = initialState.position < goalState.position ? 1 : -1;
        // flip positions/velocities based on direction
        // for arithmetic operations, will be reverted
        State initS = initialState;
        initialS.position *= acc_direction;
        initialS.velocity *= acc_direction;
        State goalS = goalState;
        goalS.position *= acc_direction;
        goalS.velocity *= acc_direction;

        // account for negaive/positive inital or final velocity
        double cutoffBegin_sec = initS.velocity / constraints.maxAcceleration;
        double cutoffBegin_dist = 0.5 * constraints.maxAcceleration * cutoffBegin_sec * cutoffBegin_sec;

        double cutoffEnd_sec = goalS.velocity / constraints.maxAcceleration;
        double cutoffEnd_dist = 0.5 * constraints.maxAcceleration * cutoffEnd_sec * cutoffEnd_sec;

        double fullTrapezoid_dist = cutoffBegin_dist + (goalS.position - initS.position) + cutoffEnd_dist;
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

        State result = initS;

        if (t < endAcc_sec)
        {
            result.position += (initS.velocity + 0.5 * constraints.maxAcceleration * time) * time;
        }
        else if (t < endFullSpeed_sec)
        {
            result.position +=
                (initS.velocity + 0.5 * constraints.maxAcceleration * endAcc_sec) * endAcc_sec +
                +((time - endAcc_sec) * constraints.maxVelocity);
        }
        else if (t <= endDecel_sec)
        {
            double timeLeft = endDecel_sec - time;
            result.position = goalS.position -
                              (goalS.velocity + timeLeft * constraints.maxAcceleration / 2.0) * timeLeft;
        }
        else
        {
            result = goalS;
        }

        return result.position * acc_direction;
    }

    State getGoalState() { return goalState; }
    State getInitialState() { return initialState; }

    void setInitialState(State initialState)
    {
        this->initialState = initialState;
    }
    void setGoalState(State goalState)
    {
        this->goalState = goalState;
    }

private:
    Constraints constraints;
    State initialState;
    State goalState;

    // TODO: put into helper class?
    template <typename T>
    int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }
};

#endif