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
};

