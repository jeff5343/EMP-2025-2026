#ifndef __PATH_PARSER__
#define __PATH_PARSER__

#include "structs/path.h"

class PathParser
{
public:
    static Path loadPath(std::string filePath);
};

#endif
