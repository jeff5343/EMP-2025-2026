#ifndef INTAKE_OUTAKE_H
#define INTAKE_OUTAKE_H
#include "vex.h"

class intake_outake
{
private:
    void intake();
    void throughtake(); // function to run intake mechanism
    
public:
    void outake();      // function to run outake mechanism
    // 2 Motor Subsystems
    void startIntakeAndThroughtake();
    void stopIntakeAndThroughtake();  // function to stop both mechanisms

    // On Startup (only set once)
    void trianglePistonIn();          // function to bring triangle piston in
    void trianglePistonOut();         // function to bring triangle piston out
    void outtakeElevationPistonIn();  // function to bring outtake elevation piston in
    void outtakeElevationPistonOut(); // function to bring outtake elevation piston out

    // Game (set multiple times in a match)
    void intakeChutePistonIn();  // function to bring intake chute piston in
    void intakeChutePistonOut(); // function to bring intake chute piston out
};

#endif