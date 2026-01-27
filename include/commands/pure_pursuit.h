#ifndef __PURE_PURSUIT_H_INCLUDED__
#define __PURE_PURSUIT_H_INCLUDED__

#include "util/structs/pid_constants.h"
#include "util/profiled_pid.h"
#include "util/pid.h"
#include "subsystems/drivetrain.h"
#include "vex.h"
#include <array>

struct Point
{
    double x;
    double y;
};

class PurePursuit
{
private:
    /*Determine Location: The vehicle figures out exactly where it is on the map (using GPS, odometry, or SLAM).
    Find the Path: It identifies the closest point on the path to its current location.
    Find the Lookahead Point: It draws a virtual circle around itself with a radius of $L$ (the Lookahead Distance).
    The point where this circle intersects with the path ahead is the Target Point.
    Transform Coordinates: The algorithm converts the coordinates of this Target Point from the "Global Map" (e.g., GPS coordinates)
    to the "Vehicle's Frame" (where the car is $(0,0)$ and facing forward).
    Calculate Curvature: It calculates the curvature of the arc required to connect the vehicle's rear axle to that Target Point.
    Steer: It turns the wheels to match that curvature.*/
    // create variables to store where you want to go, and where you are
    // variables to keep track where we are
    Pid turnPid{PidConstants{10, 0, 0}};
    Pid linearPid{PidConstants{2.5, 0, 0}};
    const double MAX_PERCENT_OUTPUT = 0.4;
    // do we need these?
    const double LOOK_AHEAD_DISTANCE = 14; // need to increase it!
    const double MAX_LINEAR_PERCENT_OUT = 20.0;
    bool backwards = false;

    int lastFoundIndex = 0;

    // TODO: tune Kp, turn up the ratio for points!

    Drivetrain &drivetrain;
    std::vector<std::array<double, 2>> path = {};

    void followGoalPoint(
        Point goalPt);
    // helper functions:
    double pt_to_pt_distance(Point p1, Point p2);
    int sgn(double num);
    Point goal_point_search();
    double distanceToGoalPt();
    void checkIfLast();

public:
    // figure out what i need to send into pure pursuit constructor??
    PurePursuit(Drivetrain &drivetrain) : drivetrain(drivetrain)
    {
        turnPid.setSetpoint(0);
        linearPid.setSetpoint(0);
    }
    void update();
    bool isAtGoal(); // new function for sequential path finding
    void setPath(const std::vector<std::array<double, 2>> &path, bool backwards)
    {
        lastFoundIndex = 0;
        this->path = path;
        this->backwards = backwards;
    }
    void reset()
    {
        lastFoundIndex = 0;
    }
};

#endif
