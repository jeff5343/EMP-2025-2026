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
    // variables to keep track where we are
    double pt_to_pt_distance;

    double current_x;
    double current_y;
    double current_heading;
    double current_velocity;

    double kP;
    int lastIndexFound = 0;
    double linear_Error = euclidianError;
    double linearVel = kP * linear_Error;
    double euclidianError;
    double turnError = min_angle(target, currentHeading);
    int lastFoundIndex;

    double target;

    double turnVel;
    double leftsidemotor = linearVel - turnVel;
    double rightsidemotor = linearVel + turnVel;

    const double kP = 3;

    // do we need these?
    const double lookaheadDistance;
    const double wheelbaseLength;

    // variables to keep track where we want to go
    double goal_x;
    double goal_y;

    // set up test path ???
    int testPath[3][2] = {{0, 0}, {0, 100}, {100, 100}};

    Drivetrain drivetrain;

    void followGoalPoint(double[2] goalPt);

    std::vector<std::array<int, 2>> path1[][2] = {{0.0, 0.0}, {0.571194595265405, -0.4277145118491421}, {1.1417537280142898, -0.8531042347260006}, {1.7098876452457967, -1.2696346390611464}, {2.2705328851607995, -1.6588899151216996}, {2.8121159420106827, -1.9791445882187304}, {3.314589274316711, -2.159795566252656}, {3.7538316863009027, -2.1224619985315876}, {4.112485112342358, -1.8323249172947023}, {4.383456805594431, -1.3292669972090994}, {4.557386228943757, -0.6928302521681386}, {4.617455513800438, 0.00274597627737883}, {4.55408382321606, 0.6984486966257434}, {4.376054025556597, 1.3330664239172116}, {4.096280073621794, 1.827159263675668}, {3.719737492364894, 2.097949296701878}, {3.25277928312066, 2.108933125822431}, {2.7154386886417314, 1.9004760368018616}, {2.1347012144725985, 1.552342808106984}, {1.5324590525923942, 1.134035376721349}, {0.9214084611203568, 0.6867933269918683}, {0.30732366808208345, 0.2295500239189426}, {-0.3075127599907512, -0.2301742560363831}, {-0.9218413719658775, -0.6882173194028102}, {-1.5334674079795052, -1.1373288016589413}, {-2.1365993767877467, -1.5584414896876835}, {-2.7180981380280307, -1.9086314914221845}, {-3.2552809639439704, -2.1153141204181285}, {-3.721102967810494, -2.0979137913841046}, {-4.096907306768644, -1.8206318841755131}};

public:
    // figure out what i need to send into pure pursuit constructor??
    PurePursuit(Drivetrain &drivetrain) : drivetrain(drivetrain)
    {
        headingController.enableContinuousInput(true);
    };

    // helper functions:
    void line_circle_intersection(double current_pt_x, double current_pt_y, double pt1[], double pt2[], const double lookaheadDistance);
    double pt_to_pt_distance(double current_pt_x, double current_pt_y, double goal_pt_x, double goal_pt_y);
    double sgn(double num);
    double goal_pt_search(double path[], double current_pt_x, double current_pt_y, const double lookaheadDistance, int lastFoundIndex);
    double find_min_angle();

    /*
    int get_target_point(double current_x, double current_y, double goal_x, double goal_y);
    int global_to_local_transform(); //definitely needs some parameters
    int calculate_steering();//definitely needs some parameters
    int update_lookahead();//optional and definitely need variables*/

    double min_angle(double target, double current_heading);

    // void setTargetPose(Pose pose);
    void update();
    // bool isAtTarget();
};

#endif