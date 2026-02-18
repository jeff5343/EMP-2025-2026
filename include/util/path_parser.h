#ifndef __PATH_PARSER__
#define __PATH_PARSER__

#include "structs/path.h"
#include "alliance.h"

class PathParser
{
public:
    static std::vector<Path> loadPaths(std::string filePath);
    static void flipForAlliance(std::vector<Path>& paths, ALLIANCE alliance);
    static constexpr double CENTIMETERS_TO_INCHES = 1 / 2.54;
};

#endif
