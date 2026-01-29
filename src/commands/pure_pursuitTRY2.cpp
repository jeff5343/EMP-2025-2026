#include "commands/pure_pursuit.h"
#include "util/math_util.h"
#include "util/angle.h"
#include <cmath>

// 2. Define the distance helper function (Euclidean distance)
double PurePursuit::pt_to_pt_distance(Point p1, Point p2)
{
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

int PurePursuit::sgn(double num)
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

Point PurePursuit::goal_point_search()
{
    // extract current X and current Y
    Pose pose = drivetrain.getPose();
    double currentX = pose.x;
    double currentY = pose.y;

    // use for loop to search intersections
    bool intersectFound = false;
    int startingIndex = lastFoundIndex;
    Point goalPt = {path[lastFoundIndex][0], path[lastFoundIndex][1]};

    for (int i = startingIndex; i < path.size() - 1; i++)
    {
        double x1 = path[i][0] - currentX;
        double y1 = path[i][1] - currentY;
        double x2 = path[i + 1][0] - currentX;
        double y2 = path[i + 1][1] - currentY;
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr = sqrt(dx * dx + dy * dy);
        double D = x1 * y2 - x2 * y1;
        double discriminant = (LOOK_AHEAD_DISTANCE * LOOK_AHEAD_DISTANCE) * (dr * dr) - (D * D);

        if (discriminant >= 0)
        {
            double sol_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / (dr * dr);
            double sol_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / (dr * dr);
            double sol_y1 = (-D * dx + fabs(dy) * sqrt(discriminant)) / (dr * dr);
            double sol_y2 = (-D * dx - fabs(dy) * sqrt(discriminant)) / (dr * dr);

            Point sol_pt1{(sol_x1 + currentX), (sol_y1 + currentY)};
            Point sol_pt2{(sol_x2 + currentX), (sol_y2 + currentY)};
            Point current{currentX, currentY};
            Point nextPoint{path[i + 1][0], path[i + 1][1]};

            // end of line-circle intersection code
            double minX = std::min(path[i][0], path[i + 1][0]);
            double minY = std::min(path[i][1], path[i + 1][1]);
            double maxX = std::max(path[i][0], path[i + 1][0]);
            double maxY = std::max(path[i][1], path[i + 1][1]);

            bool isSolPt1InRange = (minX <= sol_pt1.x && sol_pt1.x <= maxX) and (minY <= sol_pt1.y && sol_pt1.y <= maxY);
            bool isSolPt2InRange = (minX <= sol_pt2.x && sol_pt2.x <= maxX) and (minY <= sol_pt2.y && sol_pt2.y <= maxY);
            // if one or both of the solutions are in range
            if (isSolPt1InRange or isSolPt2InRange)
            {
                intersectFound = true;
                if (isSolPt1InRange and isSolPt2InRange)
                {
                    // if both solutions are in range, check which one is better
                    if (PurePursuit::pt_to_pt_distance(sol_pt1, nextPoint) < pt_to_pt_distance(sol_pt2, nextPoint))
                    {
                        goalPt = sol_pt1;
                    }
                    else
                    {
                        goalPt = sol_pt2;
                    }
                }
                else
                { // if not both solutions are in range, take the one that's in range
                    // if solution pt1 is in range, set that as goal point
                    if (isSolPt1InRange)
                    {
                        goalPt = sol_pt1;
                    }
                    else
                    {
                        goalPt = sol_pt2;
                    }
                }
            }

            // only exit loop if the solution pt found is closer to the next pt in path than the current pos
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
            intersectFound = false;
            // no new intersection found, potentially deviated from the path
            // follow path [lastFoundIndex]
            goalPt = {path[lastFoundIndex][0], path[lastFoundIndex][1]};
        }
    }
    // printf("intersect found: %s\n", intersectFound ? "true" : "false");
    return goalPt;
}

void PurePursuit::followGoalPoint(Point goalPt)
{
    Pose pose = drivetrain.getPose();

    double dx = goalPt.x - pose.x;
    double dy = goalPt.y - pose.y;

    // 1. Calculate Target Angle
    // If moving backwards, we invert the direction vector (-dy, -dx).
    // This calculates the angle for the FRONT of the robot to face AWAY from the goal.
    double absTargetAngle;
    if (backwards)
    {
        absTargetAngle = atan2(-dy, -dx);
    }
    else
    {
        absTargetAngle = atan2(dy, dx);
    }

    // Normalize angle to [0, 2PI] to match your existing logic
    if (absTargetAngle < 0)
    {
        absTargetAngle += M_PI * 2.0;
    }

    // 2. Calculate Turn Error
    double turnError = absTargetAngle - pose.radians;

    // Normalize error to [-PI, PI] for shortest turn
    if (turnError > M_PI || turnError < -M_PI)
    {
        turnError = -1 * std::copysign(1.0, turnError) * ((2 * M_PI) - std::abs(turnError));
    }

    // printf("turnError (rad): %.3f\n", turnError);

    double linearError = distanceToGoalPt();

    double turnVel = -turnPid.calculate(turnError);
    // printf("turnVel: %.3f, turnError: %.3f\n", turnVel, turnError);

    // printf("linear Error: (%.3f, %.3f)\n", linearError, linearError * kPLinear);
    // 3. Set Linear Velocity
    // If backwards, we use negative velocity.
    double linearVel = MathUtil::clamp(
        (backwards ? -1 : 1) * -linearPid.calculate(linearError),
        -MAX_LINEAR_PERCENT_OUT, MAX_LINEAR_PERCENT_OUT);

    // 4. Calculate Motor Output
    // Note: The mixing logic (L = V - T, R = V + T) usually works for reverse
    // automatically provided turnVel is calculated correctly relative to the new heading.
    double leftPercentOut = MathUtil::clamp((linearVel - turnVel) / 100.0, -MAX_PERCENT_OUTPUT, MAX_PERCENT_OUTPUT);
    double rightPercentOut = MathUtil::clamp((linearVel + turnVel) / 100.0, -MAX_PERCENT_OUTPUT, MAX_PERCENT_OUTPUT);
    // printf("percent outs: (%.3f, %.3f)\n", leftPercentOut, rightPercentOut);

    // turnVel = kP (10) * (PI/2) = 15.70
    // (20 - 15.70) = 4.3
    // (20 + 15.70) = 35.7

    // turnVel = kP (10) * (PI/4) = 7.85
    // (20 - 7.85) = 12.15
    // (20 + 7.85) = 27.85

    drivetrain.setPercentOut(leftPercentOut, rightPercentOut);
}

double PurePursuit::distanceToGoalPt()
{
    Pose pose = drivetrain.getPose();
    // current position and goal position
    Point currentPosition = {pose.x, pose.y};
    Point endPoint = {path[path.size() - 1][0], path[path.size() - 1][1]};
    double distanceToGoal = pt_to_pt_distance(currentPosition, endPoint);

    return distanceToGoal;
}

bool PurePursuit::isAtGoal()
{
    // code this function:
    //  need isAtGoal function, isAtGoal will return bool either true or false
    //  depending on how far we are to the goal
    double distanceToGoal = distanceToGoalPt();
    if (distanceToGoal <= 1.0)
        return true;
    else
        return false;
}

void PurePursuit::checkIfLast()
{
    // checks if is on last point and then stops if within 5 inches
    if (isAtGoal())
    {
        drivetrain.setPercentOut(0, 0);
    }
}

void PurePursuit::update()
{
    //:)
    Point goalPt = goal_point_search();
    Pose pose = drivetrain.getPose();
    double currentX = pose.x;
    double currentY = pose.y;
    // printf("goal: (%.3f, %.3f)\n", goalPt.x, goalPt.y);
    // printf("currentX: %.3f, currentY: %.3f\n", currentX, currentY);
    // printf("currentIndex: %d\n", lastFoundIndex);

    followGoalPoint(goalPt);
    checkIfLast();
}
