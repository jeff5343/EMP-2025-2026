#ifdef __TESTING__
#include "../../include/util/path_parser.h"
#include "../../include/util/angle.h"
#else
#include "util/path_parser.h"
#include "util/angle.h"
#endif
#include <fstream>
#include <sstream>
#include <stdlib.h>

Path PathParser::loadPath(std::string filePath)
{
    std::vector<std::array<double, 2>> points;
    double startHeading = 0;
    double lastHeading = 0;

    std::ifstream f(filePath);
    if (!f.is_open())
    {
        printf("unable to open %s!", filePath.c_str());
        return Path{{}, 0, 0};
    }

    bool pathStarts = false;
    int pointCount = 0;
    std::string line;
    while (std::getline(f, line))
    {
        if (line.find("#PATH-POINTS-START Path") != std::string::npos)
        {
            pathStarts = true;
            continue;
        }
        if (line.find("#PATH.JERRYIO-DATA") != std::string::npos)
            break;
        if (!pathStarts)
            continue;

        std::stringstream input{line};
        int i = 0;
        std::array<double, 2> nums;
        for (std::string chunk; std::getline(input, chunk, ',') && i < 2; i++)
        {
            nums[i] = atof(line.c_str());
        }

        if (!input.eof())
        {
            // add heading
            std::string heading;
            std::getline(input, heading);
            double headingRad = atof(heading.c_str());
            if (pointCount == 0)
            {
                startHeading = Angle::toRadians(headingRad);
            }
            else
            {
                lastHeading = Angle::toRadians(atof(heading.c_str()));
            }
        }
        points.push_back(nums);
        pointCount++;
    }
    f.close();
    return Path{points, startHeading, lastHeading};
}