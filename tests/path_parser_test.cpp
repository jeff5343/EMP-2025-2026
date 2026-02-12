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

    std::vector<Path> paths = pathParser.loadPaths("../Skillsauto.txt");
    pathParser.flipForAlliance(paths, ALLIANCE::RED_LEFT);

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