#ifndef __PATH_PARSER__
#define __PATH_PARSER__

#include <vector>
#include <array>
#include <string>

class PathParser
{
public:
    std::vector<std::array<double, 2>> parsePath(std::string filePath);

private:
    std::string readSDCardFile(std::string filePath);
};

#endif
