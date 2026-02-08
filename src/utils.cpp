#include "utils.hpp"
#include "Problem.hpp"
#include <cmath>

double euclideanDistance(const Point& p1, const Point& p2) {
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

bool segmentsIntersect(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
    // Compute the direction vectors of the segments
    double d1x = p2.x - p1.x;
    double d1y = p2.y - p1.y;
    double d2x = p4.x - p3.x;
    double d2y = p4.y - p3.y;

    // Compute the determinant
    double det = d1x * d2y - d1y * d2x;

    if (det == 0) {
        // The segments are parallel
        return false;
    }

    // Compute the parameters of the intersection point
    double t1 = ((p3.x - p1.x) * d2y - (p3.y - p1.y) * d2x) / det;
    double t2 = ((p3.x - p1.x) * d1y - (p3.y - p1.y) * d1x) / det;

    // Check if the intersection point is on both segments
    return (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1);
}

bool segmentIntersectsObstacle(const Point& p1, const Point& p2, const Obstacle& obs) {
    // Check if the segment intersects any of the four edges of the obstacle
    Point obsCorners[4] = {
        obs.ll_corner,
        Point(obs.ll_corner.x + obs.lx, obs.ll_corner.y),
        Point(obs.ll_corner.x + obs.lx, obs.ll_corner.y + obs.ly),
        Point(obs.ll_corner.x, obs.ll_corner.y + obs.ly)
    };

    for (int i = 0; i < 4; ++i) {
        if (segmentsIntersect(p1, p2, obsCorners[i], obsCorners[(i + 1) % 4])) {
            return true;
        }
    }
    return false;
}

bool segmentIntersectsObstacles(const Point& p1, const Point& p2, const std::vector<Obstacle>& obstacles) {
    for (const auto& obs : obstacles) {
        if (segmentIntersectsObstacle(p1, p2, obs)) {
            return true;
        }
    }
    return false;
}