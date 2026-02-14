/*
Geometric utilities.
*/

#pragma once

#include "Problem.hpp"
#include <vector>

double euclideanDistance(const Point& p1, const Point& p2);

bool pointInObstacle(const Point& p, const Obstacle& obs);
bool pointInObstacles(const Point& p, const std::vector<Obstacle>& obstacles);

bool segmentsIntersect(const Point& p1, const Point& p2, const Point& p3, const Point& p4);
bool segmentIntersectsObstacle(const Point& p1, const Point& p2, const Obstacle& obs);
bool segmentIntersectsObstacles(const Point& p1, const Point& p2, const std::vector<Obstacle>& obstacles);

double segmentCollisionDistance(const Point& p1, const Point& p2, const Obstacle& obs);
double segmentCollisionDistance(const Point& p1, const Point& p2, const std::vector<Obstacle>& obstacles);