#include "subsystems/intake_outake.h"

/* private functions */

void IntakeOuttake::intake(double percentOut)
{
    intakeMotor.spin(vex::forward, percentOut * 12.0, vex::voltageUnits::volt);
}

void IntakeOuttake::throughtake(double percentOut)
{
    throughtakeMotor.spin(vex::forward, percentOut * 12.0, vex::voltageUnits::volt);
}

void IntakeOuttake::outtake(double percentOut)
{
    outakeMotor.spin(vex::forward, percentOut * 12.0, vex::voltageUnits::volt);
}

/* public functions */

// two motor subsystems for intake and throughtake
void IntakeOuttake::startIntaking()
{
    intake(0.8);
    throughtake(0.0);
    outtake(0);
}

// two motor subsystems for intake and throughtake
void IntakeOuttake::startReverseIntaking()
{
    intake(-0.8);
    throughtake(-0.8);
    outtake(0);
}

void IntakeOuttake::startOuttakingHigh()
{
    intake(0.8);
    throughtake(0.8);
    outtake(0.8);
}

void IntakeOuttake::startOuttakingMid()
{
    intake(0.8);
    throughtake(0.8);
    outtake(-0.8);
}

void IntakeOuttake::stop()
{
    intake(0);
    throughtake(0);
    outtake(0);
}

/* pistons */

// game (set multiple times in a match)
void IntakeOuttake::intakeChutePistonIn()
{
    // insert piston code here to bring intake chute piston in
    intakeChutePiston.set(false);
}
void IntakeOuttake::intakeChutePistonOut()
{
    // insert piston code here to bring intake chute piston out
    intakeChutePiston.set(true);
}
void IntakeOuttake::intakeChutePistonToggle()
{
    intakeChuteOut = !intakeChuteOut;
    intakeChutePiston.set(intakeChuteOut);
}

// on startup (only set once)
void IntakeOuttake::trianglePistonIn()
{
    // insert piston code here to bring triangle piston in

    trianglePiston.set(false);
}
void IntakeOuttake::trianglePistonOut()
{
    // insert piston code here to bring triangle piston out
    trianglePiston.set(true);
}

void IntakeOuttake::trianglePistonToggle()
{
    triangleOut = !triangleOut;
    trianglePiston.set(triangleOut);
}

void IntakeOuttake::outtakeElevationPistonIn()
{
    // insert piston code here to bring outtake elevation piston in
    outtakeElevationPiston.set(false);
}
void IntakeOuttake::outtakeElevationPistonOut()
{
    // insert piston code here to bring outtake elevation piston out
    outtakeElevationPiston.set(true);
}

void IntakeOuttake::descorerPistonIn() {
    descorePiston.set(true);
}

void IntakeOuttake::descorerPistonOut() {
    descorePiston.set(false);
}

void IntakeOuttake::descorerPistonToggle() {
    descoreOut = !descoreOut;
    descorePiston.set(descoreOut);
}