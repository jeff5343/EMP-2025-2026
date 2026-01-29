#include "../include/util/path_parser.h"

#include <fstream>

/** Command in order to run:
 * g++ --std=c++11 path_parser_test.cpp ../src/util/path_parser.cpp -D__TESTING__ && ./a.out
 */

int main()
{
    PathParser pathParser{};

    std::ofstream fileOutput{"vector_output.txt", std::ofstream::out | std::ofstream::trunc};
    if (!fileOutput.is_open())
    {
        printf("unable to open file!");
        return -1;
    }

    std::vector<Path> paths = pathParser.loadPaths("../path1.txt");

    // negate all the values
    int neg[2] = {1, 1};
    // if (alliance == ALLIANCE::BLUE_BOT || alliance == ALLIANCE::BLUE_TOP)
    if (false)
        neg[0] = -1;
    // if (alliance == ALLIANCE::RED_BOT || alliance == ALLIANCE::BLUE_BOT)
    if (false)
        neg[1] = -1;

    for (Path &path : paths)
    {
        for (std::array<double, 2> &point : path.points)
        {
            point[0] *= neg[0];
            point[1] *= neg[1];
        }
    }

    for (Path &path : paths)
    {
        fileOutput << "startHeading: " << path.startHeadingRadians << ", endHeading: " << path.endHeadingRadians << "\n";
        fileOutput << "{\n";
        for (const auto &point : path.points)
        {
            fileOutput << "{" << point[0] << ", " << point[1] << "}\n";
        }
        fileOutput << "}\n";
    }
    fileOutput.close();

    return 0;
}