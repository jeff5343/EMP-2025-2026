#include "commands/pure_pursuit.h"
#include "util/math_util.h"
#include "util/angle.h"
#include <cmath>

double path1[][2] = {{0.0, 0.0}, {0.571194595265405, -0.4277145118491421}, {1.1417537280142898, -0.8531042347260006}, {1.7098876452457967, -1.2696346390611464}, {2.2705328851607995, -1.6588899151216996}, {2.8121159420106827, -1.9791445882187304}, {3.314589274316711, -2.159795566252656}, {3.7538316863009027, -2.1224619985315876}, {4.112485112342358, -1.8323249172947023}, {4.383456805594431, -1.3292669972090994}, {4.557386228943757, -0.6928302521681386}, {4.617455513800438, 0.00274597627737883}, {4.55408382321606, 0.6984486966257434}, {4.376054025556597, 1.3330664239172116}, {4.096280073621794, 1.827159263675668}, {3.719737492364894, 2.097949296701878}, {3.25277928312066, 2.108933125822431}, {2.7154386886417314, 1.9004760368018616}, {2.1347012144725985, 1.552342808106984}, {1.5324590525923942, 1.134035376721349}, {0.9214084611203568, 0.6867933269918683}, {0.30732366808208345, 0.2295500239189426}, {-0.3075127599907512, -0.2301742560363831}, {-0.9218413719658775, -0.6882173194028102}, {-1.5334674079795052, -1.1373288016589413}, {-2.1365993767877467, -1.5584414896876835}, {-2.7180981380280307, -1.9086314914221845}, {-3.2552809639439704, -2.1153141204181285}, {-3.721102967810494, -2.0979137913841046}, {-4.096907306768644, -1.8206318841755131}};

struct Point {
    double x;
    double y;
};

// 2. Define the distance helper function (Euclidean distance)
double pt_to_pt_distance(Point p1, Point p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.x - p1.y, 2));
}

int sgn(double num)
{
    if (num>=0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

Point point1{0,0};
double currentPos[] = {point1.x, point1.y};
double currentHeading =330;
int lastFoundIndex =0;
double lookAheadDis = 0.8;
int linearVel =100;

//if using rotations set to true
bool using_rotation = false;

//determines how long this will occur
int numOfFrames =400;

void pure_pursuit_step (double path[][2], double currentPos[], int currentHeading, double lookAheadDis, int LastFoundindex)
{
    //extract current X and current Y
    double currentX=currentPos[0];
    double currentY=currentPos[1];
    

    //use for loop to search intersections
    int lastFoundIndex =LastFoundindex;
    bool intersectFound = false;
    int startingIndex = lastFoundIndex;
    int lastIndex{};

    for (int i =0; i<lastIndex;i++)
    {
        double x1 = path [i][0] - currentX;
        double y1 = path [i][1]-currentY;
        double x2 = path [i+1][0] - currentX;
        double y2 = path [i+1][1] - currentY;
        double dx = x2-x1;
        double dy = y2-y1;
        double dr = sqrt(dx*dx + dy*dy);
        double D = x1*y2 -x2*y1; 
        double discriminant = ((lookAheadDis*lookAheadDis)* (dr*dr)*(D*D));

        if (discriminant >=0)
        {
            double sol_x1 = (D*dy + sgn(dy) * dx * sqrt (discriminant)) / (dr*dr);
            double sol_x2 = (D*dy - sgn(dy) * dx * sqrt (discriminant)) / (dr*dr);
            double sol_y1 = (-D*dx + abs(dy) * sqrt (discriminant)) / (dr*dr);
            double sol_y2 = (-D*dx - abs(dy) * sqrt (discriminant)) / (dr*dr);

            Point sol_pt1 = {(sol_x1 + currentX), (sol_y1+currentY)};
            Point sol_pt2 = {(sol_x2 + currentX), (sol_y1+currentY)};

            //end of line-circle intersection code
            double minX = std::min (path[i][0], path[i+1][0]);
            double minY = std::min (path[i][1], path[i+1][1]);
            double maxX = std::max (path[i][0], path[i][0]);
            double maxY = std::max (path[i][1], path[i+1][1]);

            //if one or both of the solutions are in range
            if ( ((minX <= sol_pt1.x<=maxX) and (minY <= sol_pt1.y <= maxY)) or ((minX<=sol_pt2.x<=maxX) and (minY <= sol_pt2.y <= maxY)))
            {
                bool foundIntersection = true;
                Point nextPoint = {path[i+1][0], path[i+1][1]};
                 Point goalPt;

                //if both solutions are in range, check which one is better
                if (pt_to_pt_distance(sol_pt1, nextPoint) < pt_to_pt_distance(sol_pt2, nextPoint))
                {
                    goalPt = sol_pt1;
                }
                else
                {
                    goalPt = sol_pt2;
                }
            }
        }
    }
}
