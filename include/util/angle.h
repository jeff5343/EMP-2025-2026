#ifndef __ANGLE_H_INCLUDED__
#define __ANGLE_H_INCLUDED__

#include <cmath>

namespace Angle
{
    /* Clamps radians between 0 and 2pi */
    inline static double wrapRadians(double radians)
    {
        double twopi = 2 * M_PI;
        return radians - (twopi * std::floor(radians / twopi));
    }

    inline static double toRadians(double degrees)
    {
        return degrees * (M_PI / 180.0);
    }

    inline static double toDegrees(double radians) {
        return radians * (180.0 / M_PI);
    }
}

#endif