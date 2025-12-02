#include "util/path_parser.h"
#include <string>
#include <fstream>

std::vector<double[2]> parsePath()
{
    std::vector<double[2]> path;

    std::ifstream f("path1.txt");

    std::string s;
    bool pathStarts = false;
    while (std::getline(f, s))
    {
        if (s.rfind("#PATH-POINTS-START Path", 0) != std::string::npos)
        {
            pathStarts = true;
            continue;
        }
        if (s.rfind("#PATH.JERRYIO-DATA"))
        {
            return;
        }
        if (!pathStarts)
        {
            continue;
        }

        std::istringstream input;
        input.str(s);
        i = 0;
        int nums[3];
        for (std::string line; std::getline(input, s, ','); i++)
        {
            nums[i] = (double)s;
        }
    
        
        
    }
}