#ifndef __SUPERSTRUCTURE_H_INCLUDED__
#define __SUPERSTRUCTURE_H_INCLUDED__

#include "vex.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"
#include "subsystems/outtake.h"
#include "subsystems/throughtake.h"

#include "subsystems/arm.h"
#include "subsystems/flap.h"
#include "subsystems/poop_chute.h"

class Superstructure
{
private:
    // subsystems
    Drivetrain drivetrain{};
    InTake intake{};
    OutTake outtake{};
    ThroughTake throughtake{};
    Arm arm{};
    Flap flap{};
    PoopChute poopChute{};

public:
    void chuteIntake();
    void groundIntake();
    void poop();
    void bottomScoring();
    void raisedScoreingPosition();
    void levelScoringPosition();
    void armScoringOut();
    void travelPosition();
    void stopAllMotors();

    void arcadeDrive(double x, double y);

    Drivetrain& getDrivetrain() {
        return drivetrain;
    }
};

#endif
