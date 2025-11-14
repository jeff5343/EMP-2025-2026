#include "subsystems/flap.h"
#include "robot_config.h"

void Flap::open() {
    armPneumatic.set(true);
}

void Flap::close() {
    armPneumatic.set(false);
}