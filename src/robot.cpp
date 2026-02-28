#include "robot.h"
#include "util/angle.h"
#include "util/path_parser.h"
#include <cmath>
#include "vex.h"

double Robot::intakeSpeed = 0;
double Robot::throughtakeSpeed = 0;
double Robot::outtakeSpeed = 0;

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
    printf("done calibrating!\n");

    Brain.Screen.clearLine();
    Brain.Screen.print("IS SD CARD LOADED???");

    paths = PathParser::loadPaths(IS_SKILLS ? skillsPathFileName : autoPathFileName);
    backwards = IS_SKILLS ? backwardsSkills : backwards;
    PathParser::flipForAlliance(paths, alliance);

    // need to get starting points of path not just the first path we need to follow
    drivetrain.resetOdometry(paths[0].points[0][0], paths[0].points[0][1], paths[0].startHeadingRadians);
    purePursuit.setPath(paths[0].points, true);

    printf("done loading %lu paths!\n", paths.size());

    isCalibrating = false;

    // TODO: uncomment
    // deploy pistons at start of the match
    //  intakeOuttake.outtakeElevationPistonOut();

    controller.ButtonR1.pressed(Robot::toggleIntake); //R1: intake 
    controller.ButtonR2.pressed(Robot::toggleReverseIntake); //R2: reverse intake
    controller.ButtonL1.pressed(Robot::toggleOuttakeHigh); //L1: Score long goal
    controller.ButtonL2.pressed(Robot::toggleOuttakeMid); //L2: score mid goal
};

void Robot::usercontrolPeriodic()
{
    /* DON'T RUN IF CALIBRATING */
    if (isCalibrating)
        return;

    intakeOuttake.set(intakeSpeed, throughtakeSpeed, outtakeSpeed);

    /* TELEOP DRIVING: */
/*
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
        intakeOuttake.stop();*/

    // A - reset odometry
    if (controller.ButtonA.pressing())
    {
         if (!hasToggledDescorePiston)
        {
            hasToggledDescorePiston = true;
            intakeOuttake.descorerPistonToggle();
        }
               // drivetrain.resetOdometry(paths[0].points[0][0], paths[0].points[0][1], paths[0].startHeadingRadians);
    } else {
        hasToggledDescorePiston = false;
    }

    // TODO: uncomment
    // Y - toggle intake chute piston
    if (controller.ButtonY.pressing())
    {
        if (!hasToggledIntakeChutePiston)
        {
            hasToggledIntakeChutePiston = true;
            intakeOuttake.intakeChutePistonToggle();
            // intakeOuttake.trianglePistonToggle();
        }
    }
    else
    {
        hasToggledIntakeChutePiston = false;
    }

    if (controller.ButtonX.pressing())
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
            timer.reset();
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

    double dRight = drivetrain.getOdometry().getDeltaRightDistInchesPerSec();
    double dBack = drivetrain.getOdometry().getDeltaBackDistInchesPerSec();
    // printf("wat: %.3f, back: %.3f\n", dRight, dBack);
    // printf("time: %d\n", timer.time());

    if (purePursuit.isAtGoal() || ((timer.time() > 200) && (std::fabs(dRight) < 0.1 && std::fabs(dBack) < 0.3)))
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
        timer.reset();
    }
}

void Robot::followPathCommand(int currentPathIndex, bool turning)
{
    // need to 1. determine path we are following
    // 2. set that path to pure pursuit
    // 3. follow that path until we reach the end and then stop

    printf("following path: %d\n", currentPathIndex);

    pathIndex = currentPathIndex;
    purePursuit.setPath(paths[pathIndex].points, backwards[pathIndex]);

    vex::timer stopwatch = vex::timer();
    double dRight = drivetrain.getOdometry().getDeltaRightDistInchesPerSec();
    double dBack = drivetrain.getOdometry().getDeltaBackDistInchesPerSec();
    // uncommented for testing...
    // if (turning)
    //     purePursuit.setTurnKPForTurnPaths();
    // else
    //     purePursuit.setTurnKPForStraightPaths();

    while (!purePursuit.isAtGoal() &&
           ((stopwatch.time() < 300) || (std::fabs(dRight) > 0.05 || std::fabs(dBack) > 0.3)))
    {
        purePursuit.update();

        printf("wat: %.3f, back: %.3f\n", dRight, dBack);
        printf("dist: %.3f\n", purePursuit.distanceToGoalPt());

        dRight = drivetrain.getOdometry().getDeltaRightDistInchesPerSec();
        dBack = drivetrain.getOdometry().getDeltaBackDistInchesPerSec();

        vex::wait(20, vex::msec);
    }
    drivetrain.setPercentOut(0, 0);

    printf("finished path, target heading...\n");

    // make sure we are facing correct end position
    headingController.goToTargetHeadingCommand(paths[currentPathIndex].endHeadingRadians);
}

void Robot::goForwardSlowly(double speed)
{
    printf("going forward...\n");
    drivetrain.setPercentOut(speed, speed);

    double dRight = drivetrain.getOdometry().getDeltaRightDistInchesPerSec();
    double dBack = drivetrain.getOdometry().getDeltaBackDistInchesPerSec();

    vex::wait(1000, vex::msec);

    while (std::fabs(dRight) > 0.05 || std::fabs(dBack) > 0.3)
    {
        // printf("wat: %.3f, back: %.3f\n", dRight, dBack);

        dRight = drivetrain.getOdometry().getDeltaRightDistInchesPerSec();
        dBack = drivetrain.getOdometry().getDeltaBackDistInchesPerSec();

        vex::wait(20, vex::msec);
    }

    drivetrain.setPercentOut(0, 0);
}

void Robot::backup(double speed)
{
    drivetrain.setPercentOut(speed, speed);
    vex::wait(500, vex::msec);
    drivetrain.setPercentOut(0, 0);
}

void Robot::autonomousIntake()
{
    intakeOuttake.intakeChutePistonOut();
    // vex:wait(1000, vex::msec);
    goForwardSlowly(0.15);
    drivetrain.setPercentOut(0.05, 0.05);
    intakeOuttake.startIntaking();
    vex::wait(5000, vex::msec);
    intakeOuttake.stop();
    drivetrain.stop();
}

void Robot::autonomousScoreLongGoal()
{
    goForwardSlowly(-0.25);
    drivetrain.setPercentOut(-.15, -.15);
    // printf("outtaking long goal...");
    intakeOuttake.startOuttakingHigh();
    vex::wait(3000, vex::msec); // wait 10 seconds to score
    intakeOuttake.stop();
    drivetrain.stop();
}

void Robot::autonomousScoreLowGoal()
{
    goForwardSlowly(0.25);
    printf("outtaking low goal...");
    intakeOuttake.startReverseIntaking();
    vex::wait(2000, vex::msec); // wait 2 seconds to score (TODO: need to time!)
    intakeOuttake.stop();
}

void Robot::autonomousPark()
{
    drivetrain.setPercentOut(1.0, 1.0);
    vex::wait(1000, vex::msec);
}

void Robot::autonomousRun1()
{
    while (isCalibrating)
    {
        vex::wait(100, vex::msec);
    }
    vex::wait(200, vex::msec);

    // go score that one ball ahahaha
    followPathCommand(0, false);
    followPathCommand(1, false);
    autonomousScoreLongGoal();

    // score round 1 (intaking and scoring)
    intakeOuttake.intakeChutePistonOut();
    followPathCommand(2, false);
    autonomousIntake();
    followPathCommand(3, false);
    autonomousScoreLongGoal();
    //add reverse intake here

    // score round 2 (intaking and scoring)
    followPathCommand(4, false);
    autonomousIntake();
    
    followPathCommand(5, false);
    autonomousScoreLongGoal();
}

void Robot::skillz()
{
    while (isCalibrating)
    {
        vex::wait(100, vex::msec);
    }
    vex::wait(200, vex::msec);
    followPathCommand(0, false); // line up to tube
    autonomousIntake();          // fill up intake

    followPathCommand(1, false); // go to long goal
    autonomousScoreLongGoal();   // go score long goal

    followPathCommand(2, false); // unstuck triangle thing by going forwards

    followPathCommand(3, false); // line up to get a couple more balls

    followPathCommand(4, false);   // get in front of balls
    vex::wait(200, vex::msec);     // make sure intake is starting
    intakeOuttake.startIntaking(); // start intaking

    followPathCommand(5, false); // get balls by driving forwards
    intakeOuttake.stop();        // stop intaking at end of path
    vex::wait(200, vex::msec);

    followPathCommand(6, false); // go backwards

    followPathCommand(7, false); // go to long goal again

    // start scoring on the other side
    followPathCommand(8, false);
    autonomousScoreLongGoal();

    // go to chute on other side (TODO: need to make sure flap is down)
    followPathCommand(9, false);
    autonomousIntake();

    backup(-.2);
    // go to target heading for path 9 (TODO: we probably need to back out before we do this...)
    headingController.goToTargetHeadingCommand(paths[10].startHeadingRadians);
    followPathCommand(10, false);
    // score low center goal
    autonomousScoreLowGoal();

    backup(-.2);
    // go to target heading to get ready to line up
    headingController.goToTargetHeadingCommand(paths[11].startHeadingRadians);
    followPathCommand(11, false);

    // line up to park
    headingController.goToTargetHeadingCommand(paths[12].startHeadingRadians);
    followPathCommand(12, false);
    // park
    followPathCommand(13, false);
    autonomousPark();
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