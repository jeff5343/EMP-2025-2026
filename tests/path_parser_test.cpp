#include "../include/util/path_parser.h"

#include <fstream>

/** Command in order to run:
 * g++ --std=c++11 path_parser_test.cpp ../src/util/path_parser.cpp -D__TESTING__ && ./a.out
 */

int main()
{
    PathParser pathParser{};

    std::vector<std::array<double, 2>> path = pathParser.parsePath("../path1.txt");
    std::ofstream fileOutput{"vector_output.txt", std::ofstream::out | std::ofstream::trunc};
    if (!fileOutput.is_open()) {
        printf("unable to open file!");
        return -1;
    }
    fileOutput << "{\n"; 
    for (const auto &point : path)
    {
        fileOutput << "{" << point[0] << ", " << point[1] << "}\n";
    }
    fileOutput << "}\n"; 
    fileOutput.close();

    return 0;
}