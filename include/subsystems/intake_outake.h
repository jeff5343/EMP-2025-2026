#ifndef INTAKE_OUTAKE_H
#define INTAKE_OUTAKE_H
#include "vex.h"

class IntakeOuttake
{
private:
    void intake(double percentOut);
    void throughtake(double percentOut); 
    void outtake(double percentOut);
    
public:
    // 3 Motor Subsystems
    void startIntaking();
    void startReverseIntaking();
    void startOuttakingHigh();
    void startOuttakingMid();
    void stop();  

    // Game (set multiple times in a match)
    void intakeChutePistonIn();  // function to bring intake chute piston in
    void intakeChutePistonOut(); // function to bring intake chute piston out
    void intakeChutPistonToggle();

    // On Startup (only set once)
    void trianglePistonIn();          // function to bring triangle piston in
    void trianglePistonOut();         // function to bring triangle piston out
    void outtakeElevationPistonIn();  // function to bring outtake elevation piston in
    void outtakeElevationPistonOut(); // function to bring outtake elevation piston out
};

#endif