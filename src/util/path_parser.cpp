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
            std::getline(input, heading, ',');
            std::getline(input, heading);
            double headingRad = atof(heading.c_str());

            headingRad = Angle::toRadians(headingRad);
            headingRad = Angle::wrapRadians(((2 * M_PI) - headingRad) + (M_PI / 2.0));
            if (points.size() == 0)
            {
                startHeading = headingRad;
            }
            else
            {
                lastHeading = headingRad;
            }
        }
        points.push_back(nums);
    }

    f.close();
    return paths;
}

void PathParser::flipForAlliance(std::vector<Path> &paths, ALLIANCE alliance)
{
    int neg[2] = {1, 1};
    if (alliance == ALLIANCE::BLUE_BOT || alliance == ALLIANCE::BLUE_TOP)
        neg[0] = -1;
    if (alliance == ALLIANCE::RED_BOT || alliance == ALLIANCE::BLUE_BOT)
        neg[1] = -1;

    for (Path &path : paths)
    {
        // flip heading on x and y axis (top to bottom)
        if (neg[1] == -1)
        {
            path.startHeadingRadians = Angle::wrapRadians(M_PI - (path.startHeadingRadians - M_PI));
            path.endHeadingRadians = Angle::wrapRadians(M_PI - (path.endHeadingRadians - M_PI));
        }

        // flip heading on y axis (red to blue)
        if (neg[0] == -1)
        {
            path.startHeadingRadians = Angle::wrapRadians(M_PI - path.startHeadingRadians);
            path.endHeadingRadians = Angle::wrapRadians(M_PI - path.endHeadingRadians);
        }

        // flip coordinates based on alliance/top/bottom
        for (std::array<double, 2> &point : path.points)
        {
            point[0] *= neg[0];
            point[1] *= neg[1];
        }
    }
}