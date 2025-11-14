#ifndef __ARM_H_INCLUDED__
#define __ARM_H_INCLUDED__

class Arm
{
public:
    void lowerPosition();
    void upperPosition();
    bool isUpperPosition();

private:
    bool isUpperPosition = false;
};

#endif