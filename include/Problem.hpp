#pragma once

#include <vector>
#include <string>


struct Point{
    double x, y;
    Point(double x = 0.0, double y = 0.0); // constructor with default values
};

struct Obstacle{Point ll_corner; double lx, ly;}; // defines a rectangular obstacle

class Problem{
public:
    double x_max, y_max; // dimensions of the environment

    Point start1, goal1; // start and goal positions for robot 1
    Point start2, goal2; // start and goal positions for robot 2

    double radius;

    std::vector<Obstacle> obstacles; // list of obstacles in the environment

    bool loadScenario(const std::string& filename); // loads problem data from a file
    bool isCollision(const Point& p1, const Point& p2) const; // checks if the line segment between p1 and p2 collides with any obstacles
    bool isCollision(const std::vector<Point>& path) const; // checks if a given path collides with any obstacles
    double collisionDistance(const std::vector<Point>& path) const; // calculates the distance travelled into obstacles for a given path
    std::vector<Point> verticesObstacles() const; // returns a vector of all the vertices of the obstacles that are not on the boundary of the environment
    std::vector<Point> pointsNearObstacles(double N) const; // returns a vector of points near obstacles. N is the approximate number of desired points.
};

