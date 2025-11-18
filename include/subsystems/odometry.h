#ifndef __ODOMETRY_H_INCLUDED__
#define __ODOMETRY_H_INCLUDED__

#ifdef __TESTING__
#include "../../tests/mock_vex.h"
#include "../util/pose.h"
#else
#include "vex.h"
#include "util/pose.h"
#endif

/**
 * Calculates position on field using 3 encoders.
 *
 * - All units are in inches
 * - two encoders on the sides and one encoder in the back
 * - Coordinate system used is based on right hand rule:
 *      X is forward, Y is sideways, Counter Clock Wise (CCW) is positive
 *      https://en.wikipedia.org/wiki/Right-hand_rule#Coordinates
 * - Good Resources on Odometry:
 *     explains the math:
 *         https://www.youtube.com/watch?v=vxSK2NYtYJQ&t=35s
 *     more specifics on implementation:
 *         http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
 *     OkapiLib odometry implementation (math is almost the same):
 *         https://github.com/purduesigbots/OkapiLib/blob/master/src/api/odometry/threeEncoderOdometry.cpp
 * */

class Odometry
{
private:
    /**
     * To tune DIST_CENTER_TO_RIGHT_WHEEL:
     *   spin robot 10 times (make sure its in the same position after spinning)
     *   read distance of right wheel traveled
     *     (dRightDist / deltaTheta) = (dRightDist / ((M_PI * 2) * 10))
     */
    // -0.491 from hand measurement
    static constexpr double DIST_CENTER_TO_RIGHT_WHEEL = 0; // whatwhjfawf
    static constexpr double DIST_CENTER_TO_BOT_WHEEL = 0;

    /* for one rotation of the heading, the scalar to be multiplied in rad */
    static constexpr double HEADING_DRIFT_SCALAR_RAD =  1.008140736;
    double totalRadians = 0;

    // distances based on encoders
    double rightDist = 0;
    double backDist = 0;
    // deltas
    double dRightDist = 0;
    double dBackDist = 0;
    // rotation from inertial sensor in radians
    double prevRotationRad = 0;

    Pose pose{0, 0, 0};
    Pose velocity{0, 0, 0}; // TODO: implement velocity?

    vex::thread worker;
    vex::mutex mutex;
    bool workerRunning;

    // vex threads uses c style threads rather than c++
    // don't know exaclty whats going on but ok
    static int vexThreadWrapper(void *param)
    {
        return static_cast<Odometry *>(param)->threadLoop();
    }

    int threadLoop()
    {
        while (workerRunning)
        {
            update();
            vex::this_thread::sleep_for(5);
        }
        return 0;
    }

    /**
     * Updates odometry by reading encoder distances and calculating pose.
     * Called every loop/tick by the worker thread
     * */
    void update();
    void updateEncoderDistances();
    void updatePose();

    /* added to simplify testing */
    void setNewEncoderDistances(double rightDist, double backDist);

public:
    static constexpr double WHEEL_RADIUS_INCHES = 0.991; // 2.75 / 2.0;
    // TODO: will remove later, but for testing the back wheel has a different radius
    static constexpr double BACK_WHEEL_RADIUS_INCHES = 1.9;

    Odometry()
    {
        reset(0, 0, 0);
    };

    /* MUST BE CALLED!!! */
    void startThread()
    {
        workerRunning = true;
        worker = vex::thread(Odometry::vexThreadWrapper, this);
    }

    /* returns calculated pose */
    Pose getPose()
    {
        mutex.lock();
        Pose newPose = pose;
        mutex.unlock();
        return newPose;
    }

    double getRightDist()
    {
        return rightDist;
    }

    double getBackDist()
    {
        return backDist;
    }

    void reset(double x, double y, double rad)
    {
        mutex.lock();
        pose = Pose{x, y, rad};
        rightEncoder.setPosition(0, vex::rotationUnits::rev);
        backEncoder.setPosition(0, vex::rotationUnits::rev);
        inertial.setHeading(rad / (M_PI * 2.0), vex::rotationUnits::rev);
        mutex.unlock();

        setNewEncoderDistances(0, 0);
        totalRadians = 0;
    }

    double getTotalRadians()
    {
        mutex.lock();
        double total = totalRadians;
        mutex.unlock();
        printf("%.3f\n", total);
        return total;
    }

    ~Odometry()
    {
        // stop worker thread
        workerRunning = false;
        worker.join();
    }
};

#endif