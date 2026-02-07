/*
Geometric utilities.
*/

#pragma once

#include "Problem.hpp"
#include <vector>

bool segmentsIntersect(Point& p1, Point& p2, Point& p3, Point& p4);
bool segmentIntersectsObstacle(Point& p1, Point& p2, const Obstacle& obs);
bool segmentIntersectsObstacles(Point& p1, Point& p2, std::vector<Obstacle>& obstacles);
