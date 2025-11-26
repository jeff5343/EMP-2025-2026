#include "commands/pure_pursuit.h"
#include "util/math_util.h"
#include "util/angle.h"
#include <cmath>

double max (double num1, double num2)
{
    if (num1>num2)
        return num1;
    else
        return num2;
}

double min(double num1, double num2)
{
    if (num1<num2)
        return num1;
    else 
        return num2;
}

int PurePursuit::get_target_point(double current_x, double current_y, double goal_x, double goal_y)
{
    /*Calc Distances: Calculate the distance from the vehicle's current position to every point in the path list.
    Find Closest: Identify the point with the minimum distance (to know where you are on the track).
    Search Forward: Starting from that closest index, iterate forward through the list until you find the first 
                    point where the distance is greater than $L$.
    Interpolate (Optional but Recommended): If the point is too far, mathematically find the intersection between the line segment 
                                            (current point to next point) and the lookahead circle.*/
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

//pt1: [x1, y1]
//pt2: [x2, y2]
void line_circle_intersection(double current_pt_x, double current_pt_y, double pt1[], double pt2[], const double lookaheadDistance)
{
    //extract x1, x2, y1, y2 from input arrays
    double x1=pt1[0];
    double y1=pt1[1];
    double x2 = pt2[0];
    double y2= pt2[1];
    //boolean variable to keep track of if intersections are found
    bool intersectFound=false;

    //output (intersections found) should be stored in arrays sol1 and sol2
    //if two solutions are the same, store teh same values in both sol1 and sol2
    
    //subtract currentX and currentY from [x1, y1] and [x2, y2] to offset the system to origin
    bool x1_offset= x1-current_pt_x;
    bool y1_offset= y1-current_pt_y;
    bool x2_offset= x2-current_pt_x;
    bool y2_offset= y2-current_pt_y;

    //calculate the discriminant using equations
    double dx = x2_offset -x1_offset;
    double dy = y2_offset-y1_offset;
    double dr= sqrt(dx*dx +dy*dy);
    double D = x1_offset*y2_offset -x2_offset*y1_offset;
    double discriminant = (lookaheadDistance*lookaheadDistance)* (dr*dr)- (D*D);

    if (discriminant>= 0)
    {
        intersectFound = true;

        //calculate the solutions
        double sol_x1 = (D*dy+sgn(dy)*dx*sqrt(discriminant)) /(dr*dr);
        double sol_x2 = (D*dy - sgn(dy)*dx*sqrt(discriminant)) /(dr*dr);
        double sol_y1 = (- D * dx + abs(dy)*sqrt(discriminant)) /(dr*dr);
        double sol_y2 = (- D * dx - abs(dy)*sqrt(discriminant)) /(dr*dr);

        //add currentx and currenty back to the solutions, offset the system back to its original position
        struct Point {
            double x, y;
        };
        
        Point sol1 = {sol_x1 + current_pt_x, sol_y1+current_pt_y};
        Point sol2 ={sol_x2+current_pt_x, sol_y2+current_pt_y};

        //find min and max x and y values
        double minX = min(x1, x2);
        double maxX= max(x1, x2);
        double minY= min(y1, y2);
        double maxY = max(y1, y2);

        //check to see if any of the two solution points are within the correct range
        //fora  solution point to be considered valid, its x value needs to be within minX
        //and its y value needs to be between minY and maxY
        //if sol1 OR sol2 are within the range, intersection is found
        if ((minX<=sol1.x<=maxX and minY<=sol1.y<=maxY) or (minX <= sol2.x <= maxX and minY <= sol2.y <= maxY))
        {
            intersectFound=true;
            if (minX<=sol1.x<=maxX and minY<= sol1.y<=maxY)
            {
                std::cout<<"solution 1 is valid";
            }
            if (minX <= sol2.x<= maxX and minY<=sol2.y<=maxY)
            {
                std::cout<<('solution 1 is valid!');
            }
        }
    }
}


struct Point {
    double x;
    double y;
};

// 2. Define the distance helper function (Euclidean distance)
double pt_to_pt_distance(Point p1, Point p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

// 3. Define a helper for the range check to keep code clean
bool is_in_range(Point p, double minX, double maxX, double minY, double maxY) {
    return (p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY);
}

//implement the goal_pt_search algorithm
double pt_to_pt_distance (double pt1 [], double pt2[])
{
    double distance= sqrt(pt2[0] -pt1[0]*pt1[0] +(pt2[1]-pt1[1]*pt1[1]));
    return distance;
}

double goal_pt_search (double path[], double current_position_x, double current_position_y, double lookaheadDistance, int lastFoundIndex)
{
    //initialize goalPt in case no intersection is found
    double goalPt = [0.0,0.0]; //????
    double x1 = 0;
    double y1 = 0;
    double x2 = 0;
    double y2 = 0;
    double dx = 0;
    double dy = 0;
    double dr = 0;
    double D = 0;
    double discriminant = 0;
    //use for loop to search intersections 
    bool intersectFound = false;
    int startingIndex=lastFoundIndex;
    for (int i=0;i<len(path)-1; i++)
    {
        double x1 = path[i][0] - current_position_x;
        double y1 = path[i][1] - current_position_y;
        double x2 = path[i+1][0] - current_position_x;
        double y2 = path[i+1][1] - current_position_y;
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr = sqrt (dx*dx + dy*dy);
        double D = x1*y2 - x2*y1;
         discriminant = (lookaheadDistance*lookaheadDistance) * (dr*dr) - D*D;
    
    if (discriminant >= 0)
    {
        double sol_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / dr*dr;
        double sol_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / dr*dr;
        double sol_y1 = (- D * dx + abs(dy) * sqrt(discriminant)) / dr*dr;
        double sol_y2 = (- D * dx - abs(dy) * sqrt(discriminant)) / dr*dr;
        double sol_pt1 = [sol_x1 + current_position_x, sol_y1 + current_position_y];
        double sol_pt2 = [sol_x2 + current_position_x, sol_y2 + current_position_y];
   // end of line-circle intersection code
            
        double minX = min(path[i][0], path[i+1][0]);
        double minY = min(path[i][1], path[i+1][1]);
        double maxX = max(path[i][0], path[i+1][0]);
        double maxY = max(path[i][1], path[i+1][1]);
    }
    // Pre-calculate validities to avoid repeating long boolean expressions
bool sol1_valid = is_in_range(sol_pt1, minX, maxX, minY, maxY);
bool sol2_valid = is_in_range(sol_pt2, minX, maxX, minY, maxY);

// If one or both of the solutions are in range
if (sol1_valid || sol2_valid) {
    intersectFound = true;
    Point goalPt;

    // If both solutions are in range, check which one is better
    if (sol1_valid && sol2_valid) {
        // Compare distance between intersections and the next point in path
        if (pt_to_pt_distance(sol_pt1, path[i + 1]) < pt_to_pt_distance(sol_pt2, path[i + 1])) {
            goalPt = sol_pt1;
        } else {
            goalPt = sol_pt2;
        }
    } 
    // If not both are in range, take the one that is
    else {
        if (sol1_valid) {
            goalPt = sol_pt1;
        } else {
            goalPt = sol_pt2;
        }
    }

    // Only exit loop if the solution pt found is closer to the next pt in path than the current pos
    Point currentPos = {currentX, currentY};
    
    if (pt_to_pt_distance(goalPt, path[i + 1]) < pt_to_pt_distance(currentPos, path[i + 1])) {
        // Update lastFoundIndex and exit
        lastFoundIndex = i;
        break; 
    } else {
        // Robot cannot find intersection in next segment, but don't go backward
        lastFoundIndex = i + 1;
    }

} 
// If no solutions are in range
else {
    intersectFound = false;
    // No new intersection found, potentially deviated from path
    // Follow path[lastFoundIndex]
    goalPt = path[lastFoundIndex];
}
           

}
}
