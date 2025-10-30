#ifndef __ROBOT_H_INCLUDED__
#define __ROBOT_H_INCLUDED__

#include "subsystems/drivetrain.h"

class Robot
{
private:
    // subsystems
    Drivetrain drivetrain{};

public:
    /* called in pre_auton */
    void init();

    /* called every 20ms in autonomous */
    void autonomousPeriodic();

    /* called every 20ms in usercontrol */
    void usercontrolPeriodic();
};

#endif