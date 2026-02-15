#include <vector>

#include "Problem.hpp"
#include "utils.hpp"
#include <fstream>
#include <iostream>
#include <string>



// Point struct methods
Point::Point(double _x, double _y) : x(_x), y(_y) {}

bool Problem::loadScenario(const std::string& filename) {
    // Open the file for reading
    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }

    // Read the contents of the scenario file and check for errors
    if (!(inputFile >> x_max >> y_max >> start1.x >> start1.y 
                    >> goal1.x >> goal1.y 
                    >> start2.x >> start2.y 
                    >> goal2.x >> goal2.y 
                    >> radius)) {
        std::cerr << "Error: Invalid file format" << std::endl;
        return false;
    }

    // Check for valid dimensions and radius
    if (x_max <= 0 || y_max <= 0) {
        std::cerr << "Error: Invalid environment dimensions" << std::endl;
        return false;
    }
    if (radius < 0) {
        std::cerr << "Error: Invalid radius" << std::endl;
        return false;
    }
    if (start1.x < 0 || start1.x > x_max || start1.y < 0 || start1.y > y_max ||
        goal1.x < 0 || goal1.x > x_max || goal1.y < 0 || goal1.y > y_max ||
        start2.x < 0 || start2.x > x_max || start2.y < 0 || start2.y > y_max ||
        goal2.x < 0 || goal2.x > x_max || goal2.y < 0 || goal2.y > y_max) {
        std::cerr << "Error: Start or goal positions are out of bounds" << std::endl;
        return false;
    }


    double x, y, lx, ly;
    while (inputFile >> x >> y >> lx >> ly) {
        if (lx <= 0 || ly <= 0) {
            std::cerr << "Error  : Invalid obstacle dimensions" << std::endl;
            return false;
        }

        if (x < 0 || x > x_max || y < 0 || y > y_max) {
            std::cerr << "Error: Obstacle position is out of bounds" << std::endl;
            return false;
        }

        if (x + lx > x_max || y + ly > y_max) {
            std::cerr << "Error: Obstacle exceeds environment bounds" << std::endl;
            return false;
        }

        Obstacle obs;
        obs.ll_corner = Point(x, y); 
        obs.lx = lx;
        obs.ly = ly;
        obstacles.push_back(obs);
    }

    if (!inputFile.eof()) {
        std::cerr << "Error: Invalid file format in obstacles" << std::endl;
        return false;
    }

    return inputFile.eof();
}

bool Problem::isCollision(const Point& p1, const Point& p2) const {
    // Check if the line segment between p1 and p2 collides with any obstacles
    return segmentIntersectsObstacles(p1, p2, obstacles);
};

bool Problem::isCollision(const std::vector<Point>& path) const {
    // Check if any segment of the path collides with obstacles
    if (path.size() < 2) {
        return false; // A path with fewer than 2 points has no segments to check
    }

    // Check each segment of the inner path for collisions
    for (size_t i = 0; i < path.size() - 1; ++i) {
        if (isCollision(path[i], path[i + 1])) {
            return true;
        }
    }
    // Check the segments from start to first waypoint and last waypoint to goal
    if (isCollision(start1, path.front()) || isCollision(path.back(), goal1)) {
        return true;
    }

    return false;
};


double Problem::collisionDistance(const std::vector<Point>& path) const {
    double total_collision_distance = 0.0;
    if (path.size() < 2) {
        return 0.0; // A path with fewer than 2 points has no segments to check
    }

    // Check each segment of the inner path for collision distance
    for (size_t i = 0; i < path.size() - 1; ++i) {
        total_collision_distance += segmentCollisionDistance(path[i], path[i + 1], obstacles);
    }
    // Check the segments from start to first waypoint and last waypoint to goal
    total_collision_distance += segmentCollisionDistance(start1, path.front(), obstacles);
    total_collision_distance += segmentCollisionDistance(path.back(), goal1, obstacles);

    return total_collision_distance;
};

std::vector<Point> Problem::verticesObstacles() const {
    // Returns a vector of all the vertices of the obstacles that are not on the boundary of the environment
    // Rather than returning the exact point, we return a point slightly outside the obstacle to ensure that it is not considered as a collision point when used as a waypoint
    std::vector<Point> vertices;
    for (const auto& obs : obstacles) {
        Point corners[4] = {
            obs.ll_corner,
            Point(obs.ll_corner.x + obs.lx, obs.ll_corner.y),
            Point(obs.ll_corner.x + obs.lx, obs.ll_corner.y + obs.ly),
            Point(obs.ll_corner.x, obs.ll_corner.y + obs.ly)
        };
        for(size_t i = 0; i < 4; ++i) {
            if(!pointOnBoundary(corners[i], x_max, y_max)) {
                Point outsidePoint = Point(corners[i].x + ((i==1 || i==2) ? 1e-4*x_max : -1e-4*x_max), corners[i].y + ((i>=2) ? 1e-4*y_max : -1e-4*y_max)); // Shift the corner point slightly outside the obstacle
                vertices.push_back(outsidePoint);
            }
        }

    }
    return vertices;
}

std::vector<Point> Problem::pointsNearObstacles(double N) const {
    // Returns a vector of points near obstacles. N is the approximate number of desired points.
    std::vector<Point> points;
    double total_perimeter = 0.0;
    for (const auto& obs : obstacles) {
        total_perimeter += 2 * (obs.lx + obs.ly);
    }
    for (const auto& obs : obstacles) {
        double perimeter = 2 * (obs.lx + obs.ly);
        int num_points = static_cast<int>(N * (perimeter / total_perimeter)); // Proportional to the perimeter of the obstacle
        for (int i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / num_points;
            Point p;
            if (t < 0.25) {
                p = Point(obs.ll_corner.x + t * 4 * obs.lx, obs.ll_corner.y - 1e-4*y_max); // Bottom edge
            } else if (t < 0.5) {
                p = Point(obs.ll_corner.x + obs.lx + 1e-4*x_max, obs.ll_corner.y + (t - 0.25) * 4 * obs.ly); // Right edge
            } else if (t < 0.75) {
                p = Point(obs.ll_corner.x + (1 - (t - 0.5) * 4) * obs.lx, obs.ll_corner.y + obs.ly + 1e-4*y_max); // Top edge
            } else {
                p = Point(obs.ll_corner.x - 1e-4*x_max, obs.ll_corner.y + (1 - (t - 0.75) * 4) * obs.ly); // Left edge
            }
            if(!pointOnBoundary(p, x_max, y_max) && !pointInObstacles(p, obstacles)) { // Ensure the point is not on the boundary and not inside any obstacle
                points.push_back(p);
            }
        }
    }
    return points;
}
    