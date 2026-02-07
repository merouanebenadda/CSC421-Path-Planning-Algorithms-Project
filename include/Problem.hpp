#pragma once

#include <vector>

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

    bool isCollision(Point& p1, Point& p2){}; // checks if the line segment between p1 and p2 collides with any obstacles
    bool isCollision(std::vector<Point>& path){}; // checks if a given path collides with any obstacles
};

