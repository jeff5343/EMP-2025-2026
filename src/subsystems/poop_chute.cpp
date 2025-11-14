#include "subsystems/poop_chute.h"
#include "robot_config.h"

void PoopChute::poop() {
    poopChute.set(true);
    isOpen = true;
}

void PoopChute::constipate() {
    poopChute.set(false);
    isOpen = false;
}

bool PoopChute::isOpened() {
    return isOpen;
}

double getHue() {
    return colorSensor.hue();
}