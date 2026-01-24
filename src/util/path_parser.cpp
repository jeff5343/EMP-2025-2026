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

std::vector<Path> PathParser::loadPaths(std::string filePath)
{
    std::vector<Path> paths;

    std::ifstream f(filePath);
    if (!f.is_open())
    {
        printf("unable to open %s!\n", filePath.c_str());
        return paths;
    }

    std::vector<std::array<double, 2>> points;
    double startHeading = 0;
    double lastHeading = 0;

    int pathCount = 0;

    std::string line;
    while (std::getline(f, line))
    {
        if (line.find("#PATH-POINTS-START") != std::string::npos)
        {
            if (pathCount > 0)
            {
                paths.push_back(Path{points, startHeading, lastHeading});
                points = {};
            }
            pathCount++;
            continue;
        }

        if (line.find("#PATH.JERRYIO-DATA") != std::string::npos)
        {
            paths.push_back(Path{points, startHeading, lastHeading});
            break;
        }

        std::stringstream input{line};
        int i = 0;
        std::array<double, 2> nums{};
        for (std::string chunk; i < 2; i++)
        {
            std::getline(input, chunk, ',');
            nums[i] = atof(chunk.c_str()) * CENTIMETERS_TO_INCHES;
        }

        if (!input.eof())
        {
            // add heading
            std::string heading;
            std::getline(input, heading);
            double headingRad = atof(heading.c_str());
            if (points.size() == 0)
            {
                startHeading = Angle::toRadians(headingRad);
            }
            else
            {
                lastHeading = Angle::toRadians(atof(heading.c_str()));
            }
        }
        points.push_back(nums);
    }

    f.close();
    return paths;
}