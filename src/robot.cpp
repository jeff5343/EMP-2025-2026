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
    PathParser::flipForAlliance(paths, alliance);

    // need to get starting points of path not just the first path we need to follow
    drivetrain.resetOdometry(paths[0].points[0][0], paths[0].points[0][1], paths[0].startHeadingRadians);
    purePursuit.setPath(paths[0].points, true);

    printf("done loading %lu paths!\n", paths.size());

    // TODO: uncomment
    // deploy pistons at start of the match
    // intakeOuttake.trianglePistonOut();
    // intakeOuttake.outtakeElevationPistonOut();
};

void Robot::usercontrolPeriodic()
{
    /* DON'T RUN IF CALIBRATING */
    if (isCalibrating)
        return;

    /* TELEOP DRIVING: */

    // R1 - intake
    if (controller.ButtonR1.pressing())
        intakeOuttake.startIntaking();
    // R2 - reverse intake (score center goal low)
    else if (controller.ButtonR2.pressing())
        intakeOuttake.startReverseIntaking();
    // L1 - score long goal
    else if (controller.ButtonL1.pressing())
        intakeOuttake.startOuttakingHigh();
    // L2 - score center goal high
    else if (controller.ButtonL2.pressing())
        intakeOuttake.startOuttakingMid();
    else
        intakeOuttake.stop();

    // A - reset odometry
    if (controller.ButtonA.pressing())
    {
        drivetrain.resetOdometry(paths[0].points[0][0], paths[0].points[0][1], paths[0].startHeadingRadians);
    }

    // TODO: uncomment
    // Y - toggle intake chute piston
    // if (controller.ButtonY.pressing()) {
    //     if (!hasToggledIntakeChutePiston) {
    //         hasToggledIntakeChutePiston = true;
    //         intakeOuttake.intakeChutePistonToggle();
    //     }
    // } else {
    //     hasToggledIntakeChutePiston = false;
    // }

    if (controller.ButtonY.pressing())
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
    else if (controller.ButtonUp.pressing())
    {
        // point up
        if (!pathFollowingStarted)
        {
            pathFollowingStarted = true;
            headingController.setTargetHeading(0);
        }
        headingController.update();
    }
    else if (controller.ButtonDown.pressing())
    {
        // point down
        if (!pathFollowingStarted)
        {
            pathFollowingStarted = true;
            headingController.setTargetHeading(M_PI);
        }
        headingController.update();
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
        printf("now backwards: %s\n", backwards[pathIndex] ? "true" : "false");
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

    // make sure we are facing correct end position
    headingController.goToTargetHeadingCommand(paths[currentPathIndex].endHeadingRadians);
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

void Robot::goForwardSlowly(double timeInMs, double speed)
{
    drivetrain.setPercentOut(speed, speed);
    vex::wait(timeInMs, vex::msec);
    drivetrain.setPercentOut(0, 0);
}

void Robot::autonomousIntake()
{
    intakeOuttake.startIntaking();
    goForwardSlowly(1000, 0.1);
    vex::wait(1000, vex::msec);
    intakeOuttake.stop();
}

void Robot::autonomousScoreLongGoal()
{
    goForwardSlowly(1000, -0.1);
    intakeOuttake.startOuttakingHigh();
    vex::wait(1000, vex::msec);
    intakeOuttake.stop();
}

void Robot::autonomousRun1()
{
    autonomousPeriodic(0); // go to tube
    // intake
    autonomousIntake();

    // go to mid scoring zone
    autonomousPeriodic(1);

    // spit out bad balls
    intakeOuttake.startReverseIntaking();
    vex::wait(1000, vex::msec);
    intakeOuttake.stop();

    // make sure we are facing 180 degrees
    headingController.goToTargetHeadingCommand(M_PI);

    // go to the scoring zone
    autonomousPeriodic(2);
    // score
    autonomousScoreLongGoal();

    // go back to the tube
    autonomousPeriodic(3);
    // intake
    autonomousIntake();

    // go to scoring zone
    autonomousPeriodic(4);
    // score
    autonomousScoreLongGoal();
}