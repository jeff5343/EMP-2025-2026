#include "subsystems/arm.h"
#include "robot_config.h"

void Arm::lowerPosition()
{
    armPneumatic.set(false);
}

void Arm::upperPosition()
{
    armPneumatic.set(true);
}