#include "commands/heading_controller.h"

void HeadingController::setTargetHeading(double radians)
{
    this->targetRadians = radians;

    Pose current = drivetrain.getPose();
    headingController.setGoal(targetRadians);
    headingController.reset(current.radians);
}

void HeadingController::update()
{
    Pose current = drivetrain.getPose();
    double turnOut = headingController.calculate(current.radians);
    if (std::fabs(turnOut) > 0.01) {
        turnOut += std::copysign(1.0, turnOut) * kS;
    }
    drivetrain.setPercentOut(-turnOut, turnOut);
}

bool HeadingController::isAtTarget()
{
    return headingController.isAtGoal();
}

void HeadingController::goToTargetHeadingCommand(double radians)
{
    setTargetHeading(radians);
    while (!isAtTarget())
    {
        update();
        vex::wait(20, vex::msec);
    }
    drivetrain.setPercentOut(0, 0);
}