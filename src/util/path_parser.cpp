#ifdef __TESTING__
#include "../../include/util/path_parser.h"
#else
#include "util/path_parser.h"
#endif
#include <fstream>
#include <sstream>

std::vector<std::array<double, 2>> PathParser::parsePath(std::string filePath)
{
    std::vector<std::array<double, 2>> path;
    std::ifstream f(filePath);

    if (!f.is_open()) {
        printf("unable to open %s!", filePath.c_str());
        return {};
    }

    std::string s;
    bool pathStarts = false;
    while (std::getline(f, s))
    {
        if (s.find("#PATH-POINTS-START Path") != std::string::npos)
        {
            pathStarts = true;
            continue;
        }
        if (s.find("#PATH.JERRYIO-DATA") != std::string::npos)
            break;
        if (!pathStarts)
            continue;

        std::stringstream input{s};
        int i = 0;
        std::array<double, 2> nums;
        for (std::string line; std::getline(input, line, ',') && i < 2; i++)
        {
            nums[i] = std::stod(s);
        }
        path.push_back(nums);
    }
    f.close();
    return path;
}

std::string PathParser::readSDCardFile(std::string filePath)
{
    std::ifstream file(filePath);
    std::string data{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    file.close();
    return data;
}