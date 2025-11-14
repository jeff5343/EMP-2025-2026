#include "subsystems/arm.h"
#include "robot_config.h"

void Arm::lowerPosition()
{
    armPneumatic.set(false);
    isUpperPosition = false;
}

void Arm::upperPosition()
{
    armPneumatic.set(true);
    isUpperPosition = true;
}

bool Arm::isAtUpperPosition()
{
    return isAtUpperPosition();
}