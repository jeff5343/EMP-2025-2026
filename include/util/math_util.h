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

    inline static double deadband(double input, double deadband)
    {
        if (std::fabs(input) <= deadband)
        {
            return 0;
        }
        // f(X) = (1st / (1 - deadband)) * (X - (sgn(X) * deadband))
        return (1.0 / (1.0 - deadband)) * (input - (std::copysign(deadband, input)));
    }

    inline static double axisPower(double input, double power)
    {
        return std::copysign(std::pow(std::fabs(input), power), input);
    }
};

#endif