#ifndef __PURE_PURSUIT_H_INCLUDED__
#define __PURE_PURSUIT_H_INCLUDED__

#include "util/pid_constants.h"
#include "util/profiled_pid.h"
#include "subsystems/drivetrain.h"
#include "vex.h"

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
        //variables to keep track where we are
    double pt_to_pt_distance;
  
    double current_x;
    double current_y;
    double current_heading;
    double current_velocity;

    double kP;
    int lastIndexFound = 0;
    double linear_Error= euclidianError;
    double linearVel= kP * linear_Error;
    double euclidianError;
    double turnError = min_angle(target, currentHeading);
    int lastFoundIndex;

    double target;

    double turnVel;
    double leftsidemotor = linearVel-turnVel;
    double rightsidemotor = linearVel+turnVel;



    //do we need these?
    const double lookaheadDistance;
    const double wheelbaseLength;

    //variables to keep track where we want to go
    double goal_x;
    double goal_y;



    //set up test path ???
    int testPath [3][2] = {{0,0}, {0,100}, {100, 100}};


    

public:
    //figure out what i need to send into pure pursuit constructor??
    PurePursuit(Drivetrain &drivetrain, PidConstants headingPidConstants, double headingSetpointTolerance,
             PidConstants straightPidConstants, double straightSetpointTolerance,
             TrapezoidProfile::Constraints headingProfileConstraints,
             TrapezoidProfile::Constraints straightProfileConstraints)
        : drivetrain(drivetrain), headingController(headingPidConstants, headingProfileConstraints, headingSetpointTolerance),
          straightController(straightPidConstants, straightProfileConstraints, straightSetpointTolerance)
    {
        headingController.enableContinuousInput(true);
    };

    //helper functions:
    void line_circle_intersection(double current_pt_x, double current_pt_y, double pt1[], double pt2[], const double lookaheadDistance);
    double pt_to_pt_distance(double current_pt_x, double current_pt_y, double goal_pt_x, double goal_pt_y);
    double sgn (double num);
    double goal_pt_search(double path[], double current_pt_x, double current_pt_y, const double lookaheadDistance, int lastFoundIndex);

    /*
    int get_target_point(double current_x, double current_y, double goal_x, double goal_y);
    int global_to_local_transform(); //definitely needs some parameters
    int calculate_steering();//definitely needs some parameters
    int update_lookahead();//optional and definitely need variables*/


    double min_angle(double target, double current_heading );

    //void setTargetPose(Pose pose);
    void update();
    bool isAtTarget();
};

#endif