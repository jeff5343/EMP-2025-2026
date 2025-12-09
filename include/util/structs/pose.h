#ifndef __POSE_H_INCLUDED__
#define __POSE_H_INCLUDED__

#include <math.h>
#include <string>
#include <cstdio>

struct Pose
{
    double x;
    double y;
    double radians;

    void print()
    {
        printf("Pose(x: %.3f, y: %.3f, deg: %.2f)\n", x, y, radians * (180.0 / M_PI));
    }
};

#endif