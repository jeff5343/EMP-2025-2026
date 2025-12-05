#include "commands/pure_pursuit.h"
#include "util/math_util.h"
#include "util/angle.h"
#include <cmath>

struct Point
{
    double x;
    double y;
};

// 2. Define the distance helper function (Euclidean distance)
double pt_to_pt_distance(Point p1, Point p2)
{
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

int sgn(double num)
{
    if (num >= 0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

Point point1{0, 0};
double currentPos[] = {point1.x, point1.y};
double currentHeading = 330;
int lastFoundIndex = 0;
double lookAheadDis = 0.8;
int linearVel = 100;

// if using rotations set to true
bool using_rotation = false;

// determines how long this will occur
int numOfFrames = 400;

Point pure_pursuit_step(double path[][2], double currentPos[], int currentHeading, double lookAheadDis, int LastFoundindex)
{
    // extract current X and current Y
    double currentX = currentPos[0];
    double currentY = currentPos[1];

    // use for loop to search intersections
    int lastFoundIndex = LastFoundindex;
    bool intersectFound = false;
    int startingIndex = lastFoundIndex;
    int lastIndex{};
    Point goalPt;

    for (int i = 0; i < lastIndex; i++)
    {
        double x1 = path[i][0] - currentX;
        double y1 = path[i][1] - currentY;
        double x2 = path[i + 1][0] - currentX;
        double y2 = path[i + 1][1] - currentY;
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr = sqrt(dx * dx + dy * dy);
        double D = x1 * y2 - x2 * y1;
        double discriminant = ((lookAheadDis * lookAheadDis) * (dr * dr) * (D * D));

        bool foundIntersection;

        if (discriminant >= 0)
        {
            double sol_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / (dr * dr);
            double sol_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / (dr * dr);
            double sol_y1 = (-D * dx + abs(dy) * sqrt(discriminant)) / (dr * dr);
            double sol_y2 = (-D * dx - abs(dy) * sqrt(discriminant)) / (dr * dr);

            Point sol_pt1 = {(sol_x1 + currentX), (sol_y1 + currentY)};
            Point sol_pt2 = {(sol_x2 + currentX), (sol_y1 + currentY)};

            // end of line-circle intersection code
            double minX = std::min(path[i][0], path[i + 1][0]);
            double minY = std::min(path[i][1], path[i + 1][1]);
            double maxX = std::max(path[i][0], path[i][0]);
            double maxY = std::max(path[i][1], path[i + 1][1]);

            Point nextPoint = {path[i + 1][0], path[i + 1][1]};

            // if one or both of the solutions are in range
            if (((minX <= sol_pt1.x <= maxX) and (minY <= sol_pt1.y <= maxY)) or ((minX <= sol_pt2.x <= maxX) and (minY <= sol_pt2.y <= maxY)))
            {
                foundIntersection = true;

                // if both solutions are in range, check which one is better
                if (pt_to_pt_distance(sol_pt1, nextPoint) < pt_to_pt_distance(sol_pt2, nextPoint))
                {
                    goalPt = sol_pt1;
                }
                else
                {
                    goalPt = sol_pt2;
                }
            }
            // if not both solutions are in range, take the one that's in range
            else
            {
                // if solution pt1 is in range, set that as goal point
                if ((minX <= sol_pt1.x <= maxX) and (minY <= sol_pt1.y <= maxY))
                {
                    goalPt = sol_pt1;
                }
                else
                {
                    goalPt = sol_pt2;
                }
            }
            // only exit loop if the solution pt found is closer to the next pt in path than the current pos
            Point current{currentX, currentY};
            if (pt_to_pt_distance(goalPt, nextPoint) < pt_to_pt_distance(current, nextPoint))
            {
                // update lastFoundIndex and exit
                lastFoundIndex = i;
                break;
            }

            else
            {
                // in case for some reason the robot cannot find intersection in the next path,
                // but we also don't want it to go backward
                lastFoundIndex = i + 1;
            }
        }
        else
        {
            foundIntersection = false;
            // no new intersection found, potentially deviated from the path
            // follow path [lastFoundIndex]
            goalPt = {path1[lastFoundIndex][0], path1[lastFoundIndex][1]};
        }
    }
    return goalPt;
}

void PurePursuit::followGoalPoint(double[2] goalPt)
{
    Pose pose = drivetrain.getPose();

    double absTargetAngle = atan2(goalPt[1] - pose.y, goalPt[0] - pose.x);
    if (absTargetAngle < 0)
    {
        absTargetAngle += M_PI * 2.0;
    }

    double turnError = absTargetAngle - pose.radians;
    if (turnError > M_PI || turnError < -M_PI)
    {
        turnError = -1 * std::copysign(1.0, turnError) * (M_PI std::abs(turnError));
    }
    double linearError = std::sqrt(std::pow(goalPt[1] - pose.y, 2) +
                                   std::pow(goalPt[0] - pose.x, 2));

    double turnVel = kP * turnError;
    double linearVel = kP * linearError;
    drivetrain.setPercentOut(linearVel - turnVel, linearVel + turnVel);
}