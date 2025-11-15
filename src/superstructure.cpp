#include "vex.h"
#include "superstructure.h"
void Superstructure::Intake()
{
    intake.intake();
    throughtake.intake();
    outtake.intake();
};
void Superstructure::poop()
{
    intake.intake();
    poopChute.poop();


};
void Superstructure::bottomScoring()
{
    intake.reverse();
    throughtake.reverse();
    outtake.eject();
};
void Superstructure::raisedScoreingPosition()
{
    //Jefferson do I include: isAtUpperPosition() as well?
    arm.upperPosition();
    //arm.isAtUpperPosition();

};
void Superstructure::levelScoringPosition()
{
    arm.lowerPosition();
};
void Superstructure::armScoringOut()
{
    intake.intake();
    throughtake.intake();
    outtake.intake();
    
};
void Superstructure::travelPosition()
{

};
void Superstructure::stopAllMotors()
{

};

void Superstructure::arcadeDrive(double x, double y) {
    drivetrain.arcadeDrive(x, y);
}