#ifndef __POOP_CHUTE_H_INCLUDED__
#define __POOP_CHUTE_H_INCLUDED__

class PoopChute {
public:
    void poop();
    void constipate();
    bool isOpened();
    double getHue();

private:
    bool isOpen = false;
};

#endif