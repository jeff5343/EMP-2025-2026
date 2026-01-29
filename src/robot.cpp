#include "robot.h"
#include "util/angle.h"
#include "util/path_parser.h"
#include <cmath>
#include "vex.h"

void Robot::init(ALLIANCE alliance)
{
    /* CALIBRATE INERTIAL SENSOR */
    isCalibrating = true;
    inertial.calibrate();
    while (inertial.installed() && inertial.isCalibrating())
    {
        Brain.Screen.clearLine();
        Brain.Screen.print("Calibrating inertial sensor...");
        printf("calibrating...\n");
        vex::this_thread::sleep_for(100);
    }
    inertial.resetHeading();
    drivetrain.startOdometry();
    isCalibrating = false;
    printf("done calibrating!\n");

    paths = PathParser::loadPaths(pathFileName);
    double startX = paths[0].points[0][0];
    double startY = paths[0].points[0][1];

    drivetrain.resetOdometry(startX, startY, M_PI); // need to get starting points of path not just the first path we need to follow
    purePursuit.setPath(paths[0].points, true);

    // flip coordinates based on alliance
    // int neg[2] = {1, 1};
    // if (alliance == ALLIANCE::BLUE_BOT || alliance == ALLIANCE::BLUE_TOP)
    //     neg[0] = -1;
    // if (alliance == ALLIANCE::RED_BOT || alliance == ALLIANCE::BLUE_BOT)
    //     neg[1] = -1;

    // for (Path &path : paths)
    // {
    //     for (std::array<double, 2> &point : path.points)
    //     {
    //         point[0] *= neg[0];
    //         point[1] *= neg[1];
    //     }
    // }

    printf("done loading %lu paths!\n", paths.size());
};

void Robot::usercontrolPeriodic()
{
    /* DON'T RUN IF CALIBRATING */
    if (isCalibrating)
        return;

    /* TELEOP DRIVING: */
    if (controller.ButtonR1.pressing())
    {
        intakeMotor.spin(vex::forward, 10, vex::voltageUnits::volt);
        throughtakeMotor.spin(vex::forward, 10, vex::voltageUnits::volt);
    }
    else if (controller.ButtonL1.pressing())
    {
        intakeMotor.spin(vex::forward, -10, vex::voltageUnits::volt);
        throughtakeMotor.spin(vex::forward, -10, vex::voltageUnits::volt);}
    else
    {
        intakeMotor.spin(vex::forward, 0, vex::voltageUnits::volt);
        throughtakeMotor.spin(vex::forward, 0, vex::voltageUnits::volt);
    }

    // TODO: find out how to bind functions to events??
    if (controller.ButtonA.pressing())
    {
        drivetrain.resetOdometry(paths[0].points[0][0], paths[0].points[0][1], M_PI);
    }

    if (controller.ButtonB.pressing())
    {
        if (!pathFollowingStarted)
        {
            pidDrive.setTargetPose(poseSetpoints[poseSetpointIndex]);
            pathFollowingStarted = true;
        }
        pidDrive.update();
    }
    else if (controller.ButtonY.pressing())
    {
        if (!pathFollowingStarted)
        {
            pathFollowingStarted = true;
            pathIndex = 0;
            if (paths.size() > 0)
            {
                purePursuit.setPath(paths[pathIndex].points, backwards[pathIndex]);
            }
            purePursuit.reset();
        }
        followPaths();
    }
    else
    {
        // so that setTargetPose is ran when b is pressed
        pathFollowingStarted = false;

        // convert axis positions to range -1.0 to 1.0
        double x = static_cast<double>(controller.Axis1.position()) / 100.0;
        double y = static_cast<double>(controller.Axis3.position()) / 100.0;

        // printf("ux: %.3f, uy: %.3f", x, y);

        // apply deadband
        double deadband = 0.03;
        x = MathUtil::axisPower(MathUtil::deadband(x, deadband), 1.3);
        y = MathUtil::axisPower(MathUtil::deadband(y, deadband), 1.3);

        double staticFriction = 0.03;
        if (std::fabs(x) > 0)
            x += std::copysign(staticFriction, x);
        if (std::fabs(y) > 0)
            y += std::copysign(staticFriction, y);

        // printf("x: %.3f, y: %.3f\n", x, y);

        drivetrain.arcadeDrive(MathUtil::deadband(x, deadband), MathUtil::deadband(y, deadband));
    }

    /* logging */
    log();
}

void Robot::followPaths()
{
    if (pathIndex >= paths.size())
    {
        drivetrain.setPercentOut(0, 0);
        return;
    }

    purePursuit.update();

    if (purePursuit.isAtGoal())
    {
        printf("path: %d complete\n", pathIndex);
        pathIndex++;
        if (pathIndex >= paths.size())
        {
            drivetrain.setPercentOut(0, 0);
            return;
        }
        purePursuit.setPath(paths[pathIndex].points, backwards[pathIndex]);
    }
}

void Robot::autonomousPeriodic(int currentPathIndex)
{
    // need to 1. determine path we are following
    // 2. set that path to pure pursuit
    // 3. follow that path until we reach the end and then stop

    pathIndex = currentPathIndex;
    if (paths.size() > 0)
    {
        purePursuit.setPath(paths[pathIndex].points, backwards[pathIndex]);
    }

    while (!purePursuit.isAtGoal())
    {
        purePursuit.update();

        vex::wait(20, vex::msec);
    }
}

void Robot::log()
{
    /* subsystem logging */
    // drivetrain.log();

    /* brain logging */
    Brain.Screen.clearLine();

    // brain.Screen.print("rightD: ");
    // brain.Screen.print(drivetrain.getOdometry().getRightDist());
    Brain.Screen.print(", heading: ");
    Brain.Screen.print(Angle::toDegrees(drivetrain.getPose().radians));
    Brain.Screen.print(", x: ");
    Brain.Screen.print(drivetrain.getPose().x);
    Brain.Screen.print(", y: ");
    Brain.Screen.print(drivetrain.getPose().y);
    // brain.Screen.print(", total deg: ");
    // brain.Screen.print(Angle::toDegrees(drivetrain.getOdometry().getTotalRadians()));
}

void Robot::autonomousRun1()
{
    // add intake code here between paths
    autonomousPeriodic(0); // go to tube
    // add intake code here between paths

    vex::wait(200, vex::msec); // wait to fully intake balls
    autonomousPeriodic(1);     // sort bad balls
    // add reverse intake to spit out balls here between paths

    vex::wait(200, vex::msec); // wait to fully spit out balls
    autonomousPeriodic(2);     // go to the scoring zone
    // add outtake motors here between paths

    autonomousPeriodic(3); // go back to the tube
    // add intake code here between paths
    vex::wait(200, vex::msec); // wait to fully intake balls

    autonomousPeriodic(4); // go to scoring zone
    // add outtake motors here between paths
    vex::wait(200, vex::msec); // wait to fully outtake balls
}