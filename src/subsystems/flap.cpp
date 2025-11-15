#include "subsystems/flap.h"
#include "robot_config.h"

void Flap::open()
{
    armPneumatic.set(true);
    isOpen = true;
}

void Flap::close()
{
    armPneumatic.set(false);
    isOpen = false;
}

bool Flap::isOpened()
{
    return isOpen;
}