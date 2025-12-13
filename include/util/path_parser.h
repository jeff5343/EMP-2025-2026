#ifndef __PATH_PARSER__
#define __PATH_PARSER__

#include "structs/path.h"

class PathParser
{
public:
    static std::vector<Path> loadPaths(std::string filePath);
};

#endif
