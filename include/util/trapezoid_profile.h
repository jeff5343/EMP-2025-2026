#ifndef __TRAPEZOID_PROFILE_H_INCLUDED__
#define __TRAPEZOID_PROFILE_H_INCLUDED__

#include <cmath>

/**
 * good overview:
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
        // account for nonzero inital or final velocity
        double cutoffBegin_sec = initialState.velocity / constraints.maxAcceleration;
        double cutoffBegin_dist = 0.5 * constraints.maxAcceleration * cutoffBegin_sec * cutoffBegin_sec;
        double cutoffEnd_sec = goalState.velocity / constraints.maxAcceleration;
        double cutoffEnd_dist = 0.5 * constraints.maxAcceleration * cutoffEnd_sec * cutoffEnd_sec;

        double dist_total = cutoffBegin_dist + std::fabs(goalState.position - initialState.position) + cutoffEnd_dist;
        double t_acc = constraints.maxVelocity / constraints.maxAcceleration;
        double t_vel = (dist_total / constraints.maxVelocity) - t_acc;

        if (t_vel < 0)
            t_acc = sqrt(dist_total / constraints.maxVelocity);

        double pos = initialState.position;
        if (t_vel >= 0)
        {
            if (time >= t_vel)
                pos += constraints.maxVelocity * time * (0.5 * constraints.maxAcceleration * (time * time));
            if (time >= t_acc)
                pos += constraints.maxVelocity * (time - t_acc);
        }
        else
        {
            if (time >= t_acc)
            {
                double maxVelocityReached = t_acc * constraints.maxAcceleration;
                pos += maxVelocityReached * time - (0.5 * constraints.maxAcceleration * (time * time));
            }
        }
        pos += 0.5 * constraints.maxAcceleration * (time * time);

        return pos;
    }

    State getGoalState() { return goalState; }
    State getInitialState() { return initialState; }

    void setStartState(State initialState)
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
};

#endif