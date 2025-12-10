#ifndef PATH_H
#define PATH_H

#include <vector>
#include <array>

struct Path
{
    std::vector<std::array<double, 2>> points;
    double startHeadingRadians;
    double endHeadingRadians;
};

#endif