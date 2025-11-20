#include "vex.h"
#include "superstructure.h"

void Superstructure::chuteIntake()
{
    intake.intake();
    throughtake.intake();
    outtake.intake();
    intake.clamp();
}

void Superstructure::groundIntake()
{
    intake.intake();
    throughtake.intake();
    outtake.intake();
    intake.unclamp();
}

void Superstructure::poop()
{
    intake.intake();
    poopChute.poop();
}

void Superstructure::bottomScoringOut()
{
    intake.reverse();
    throughtake.reverse();
    outtake.eject();
}

void Superstructure::raisedScoreingPosition()
{
    arm.upperPosition();
}

void Superstructure::levelScoringPosition()
{
    arm.lowerPosition();
};
void Superstructure::armScoringOut()
{
    intake.intake();
    throughtake.intake();
    outtake.intake();
    flap.open(); // add arm
}

void Superstructure::travelPosition()
{
    flap.close();
    arm.lowerPosition();
}

void Superstructure::stopAllMotors()
{
    intake.stop();
    throughtake.stop();
    outtake.stop();
};

void Superstructure::arcadeDrive(double x, double y)
{
    drivetrain.arcadeDrive(x, y);
}