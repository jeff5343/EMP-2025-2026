//subsystems for intake and outake mechanisms
#include "subsystems/intake_outake.h"
void intake_outake::intake()
{
    //insert motors here to run intake mechanism
    
    intakeMotor.spin(vex::forward, 10, vex::voltageUnits::volt);
    
}
void intake_outake::throughtake()
{
    //insert motors here to run throughtake mechanism
    throughtakeMotor.spin(vex::forward, 10, vex::voltageUnits::volt);
}

//public functions

void intake_outake::outake()
{
    //insert motors here to run outake mechanism which also includes the 
    //throughtake and intake mechanisms
    outakeMotor.spin(vex::forward, 10, vex::voltageUnits::volt);
    intake();
    throughtake();
}

//two motor subsystems for intake and throughtake
void intake_outake::startIntakeAndThroughtake()
{
    intake();
    throughtake();
}
void intake_outake::stopIntakeAndThroughtake()
{
    //insert motors here to stop both mechanisms
    intakeMotor.spin(vex::forward, 0, vex::voltageUnits::volt);
    throughtakeMotor.spin(vex::forward, 0, vex::voltageUnits::volt);
}

//on startup (only set once)
void intake_outake::trianglePistonIn()
{
    //insert piston code here to bring triangle piston in
    
    trianglePiston.set(true);
}
void intake_outake::trianglePistonOut()
{
    //insert piston code here to bring triangle piston out 
    trianglePiston.set(false);
}
void intake_outake::outtakeElevationPistonIn()
{
    //insert piston code here to bring outtake elevation piston in
    outtakeElevationPiston.set(true);
}
void intake_outake::outtakeElevationPistonOut()
{
    //insert piston code here to bring outtake elevation piston out
    outtakeElevationPiston.set(false);
}

//game (set multiple times in a match)
void intake_outake::intakeChutePistonIn()
{
    //insert piston code here to bring intake chute piston in
    intakeChutePiston.set(true);
}
void intake_outake::intakeChutePistonOut()
{
    //insert piston code here to bring intake chute piston out
    intakeChutePiston.set(false);
}