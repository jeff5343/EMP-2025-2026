#ifdef __TESTING__
#include "../../include/subsystems/odometry.h"
#include "../../include/util/angle.h"
#else
#include "subsystems/odometry.h"
#include "util/angle.h"
#endif

void Odometry::updateEncoderDistances()
{
    double wheelCircumference = 2 * M_PI * WHEEL_RADIUS_INCHES;

    mutex.lock();
    double rightD = rightEncoder.position(vex::rev) * wheelCircumference;
    double backD = backEncoder.position(vex::rev) * wheelCircumference;
    mutex.unlock();

    setNewEncoderDistances(rightD, backD);
}

void Odometry::setNewEncoderDistances(double rightD, double backD)
{
    mutex.lock();
    // update deltas
    dRightDist = rightD - rightDist;
    dBackDist = backD - backDist;

    // update distances of encoders
    rightDist = rightD;
    backDist = backD;
    mutex.unlock();
}

void Odometry::updatePose()
{
    mutex.lock();
    // find delta heading
    double newRotationRad = (inertial.rotation(vex::rotationUnits::rev) * 2 * M_PI);
    double deltaTheta = newRotationRad - prevRotationRad;
    prevRotationRad = newRotationRad;

    // for logging
    totalRadians -= deltaTheta;

    // apply drift scalar
    deltaTheta *= HEADING_DRIFT_SCALAR_RAD;

    // double dBDist = dBackDist;
    double dBDist = 0; // for testing, make sure DIST_TO_BOT_WHEEL is 0 as well
    double dRDist = dRightDist;
    Pose currentPose = pose;
    mutex.unlock();

    // (used for debugging)
    // the distance the back wheel should have traveled if there was no drift
    // double backWheelNormalTravelDistance = (-deltaTheta * DIST_CENTER_TO_BOT_WHEEL);
    // printf(" backWheelDrift: %.3f\n", dBDist - backWheelNormalTravelDistance);

    // calculate relative distance traveled
    //  X axis is relative front and back movement
    //  Y axis is relative side to side movement
    double relYDist, relXDist;
    if (deltaTheta == 0)
    {
        // heading has not changed
        relYDist = dBDist;
        relXDist = dRDist;
    }
    else
    {
        // heading has changed, formulas to find distance
        // traveled using the arcs traveled by the encoders
        double cof = 2.0 * sin(deltaTheta / 2.0);
        relYDist = cof * ((dBDist / deltaTheta) +
                          (DIST_CENTER_TO_BOT_WHEEL));
        relXDist = cof * ((dRDist / deltaTheta) +
                          DIST_CENTER_TO_RIGHT_WHEEL);
    }

    // convert relative vector into global vector
    double relativeMagnitude = sqrt((relYDist * relYDist) + (relXDist * relXDist));
    double relativeHeading = atan2(relXDist, relYDist);

    // subtract (current heading of robot + the change in heading)
    relativeHeading -= (currentPose.radians + (deltaTheta / 2.0));

    double dY = cos(relativeHeading) * relativeMagnitude;
    double dX = sin(relativeHeading) * relativeMagnitude;

    // check for invalid values
    if (std::isnan(dY))
        dY = 0;
    if (std::isnan(dX))
        dX = 0;
    if (std::isnan(deltaTheta))
        deltaTheta = 0;

    mutex.lock();
    // update pose object
    pose.x -= dX;
    pose.y -= dY;
    // subtracting instead of adding so CCW+ and CW-
    pose.radians -= deltaTheta;
    // wrap radians
    pose.radians = Angle::wrapRadians(pose.radians);
    mutex.unlock();

    // printf("%.3f\n", dRightDist - (-deltaTheta * DIST_CENTER_TO_RIGHT_WHEEL));
    // printf("DIST_TO_RIGHT: %.3f\n", DIST_CENTER_TO_RIGHT_WHEEL);
}

void Odometry::update()
{
    updateEncoderDistances();
    updatePose();
}