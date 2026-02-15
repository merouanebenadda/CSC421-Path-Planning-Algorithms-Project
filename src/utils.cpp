#include "utils.hpp"
#include "Problem.hpp"
#include <algorithm>
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

bool pointInObstacle(const Point& p, const Obstacle& obs) {
    return (p.x >= obs.ll_corner.x && p.x <= obs.ll_corner.x + obs.lx &&
            p.y >= obs.ll_corner.y && p.y <= obs.ll_corner.y + obs.ly);
}

bool pointInObstacles(const Point& p, const std::vector<Obstacle>& obstacles) {
    for (const auto& obs : obstacles) {
        if (pointInObstacle(p, obs)) {
            return true;
        }
    }
    return false;
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

// Computes the analytical distance that the line segment from p1 to p2 travels into the obstacle obs using the Liang-Barsky algorithm
double segmentCollisionDistance(const Point& p1, const Point& p2, const Obstacle& obs) {
    const double xmin = obs.ll_corner.x;
    const double xmax = obs.ll_corner.x + obs.lx;
    const double ymin = obs.ll_corner.y;
    const double ymax = obs.ll_corner.y + obs.ly;

    const double dx = p2.x - p1.x;
    const double dy = p2.y - p1.y;
    const double seg_len = std::sqrt(dx * dx + dy * dy); 
    const double eps = 1e-12;

    if (seg_len <= eps) {
        return 0.0;
    }

    double t0 = 0.0;
    double t1 = 1.0;

    auto clip = [&](double p, double q) -> bool {
        if (std::abs(p) <= eps) {
            return q >= -eps;
        }
        double r = q / p;
        if (p < 0.0) {
            if (r > t1 + eps) {
                return false;
            }
            if (r > t0) {
                t0 = r;
            }
        } else {
            if (r < t0 - eps) {
                return false;
            }
            if (r < t1) {
                t1 = r;
            }
        }
        return true;
    };

    if (!clip(-dx, p1.x - xmin)) {
        return 0.0;
    }
    if (!clip(dx, xmax - p1.x)) {
        return 0.0;
    }
    if (!clip(-dy, p1.y - ymin)) {
        return 0.0;
    }
    if (!clip(dy, ymax - p1.y)) {
        return 0.0;
    }

    if (t1 < t0) {
        return 0.0;
    }

    const double t_start = std::max(0.0, t0);
    const double t_end = std::min(1.0, t1);
    if (t_end < t_start) {
        return 0.0;
    }

    return (t_end - t_start) * seg_len;
}

double segmentCollisionDistance(const Point& p1, const Point& p2, const std::vector<Obstacle>& obstacles) {
    double total_collision_distance = 0.0;
    for (const auto& obs : obstacles) {
        total_collision_distance += segmentCollisionDistance(p1, p2, obs);
    }
    return total_collision_distance;
}

bool pointOnBoundary(const Point& p, double x_max, double y_max) {
    return (std::abs(p.x) <= 1e-12 * x_max || std::abs(p.x - x_max) <= 1e-12 * x_max ||
            std::abs(p.y) <= 1e-12 * y_max || std::abs(p.y - y_max) <= 1e-12 * y_max);
}