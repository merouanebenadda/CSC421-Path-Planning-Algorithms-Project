#include <vector>

#include "Problem.hpp"
#include "utils.hpp"

// Point struct methods
Point::Point(double _x, double _y) : x(_x), y(_y) {}

bool Problem::isCollision(Point& p1, Point& p2) {
    // Check if the line segment between p1 and p2 collides with any obstacles
    return segmentIntersectsObstacles(p1, p2, obstacles);
};

bool Problem::isCollision(std::vector<Point>& path) {
    // Check if any segment of the path collides with obstacles
    for (size_t i = 0; i < path.size() - 1; ++i) {
        if (isCollision(path[i], path[i + 1])) {
            return true;
        }
    }
    return false;
};


