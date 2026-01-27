#ifndef __MATH_UTIL_H_INCLUDED__
#define __MATH_UTIL_H_INCLUDED__

#include <algorithm>

class MathUtil
{
public:
//gets a value from minimum to high and recieves a input and sees if it is between numbers 
    inline static double clamp(double value, double low, double high)
    {
        return std::max(low, std::min(value, high));
    }

    inline static double inputModulus(double input, double minimumInput, double maximumInput)
    {
        double modulus = maximumInput - minimumInput;

        int numMax = (int)((input - minimumInput) / modulus);
        input -= numMax * modulus;

        int numMin = (int)((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }
};

#endif