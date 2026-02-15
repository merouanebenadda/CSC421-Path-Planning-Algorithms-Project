#pragma once

#include <vector>
#include <string>

#include "Problem.hpp"


struct Tree{
    std::vector<Point> vertices;
    std::vector<int> parents; // parents[i] gives the index of the parent of vertices[i]
    std::vector<double> costs; // costs[i] gives the cost from the root to vertices[i]

    Tree(Point root); // Initializes the tree with the start point
};

class RRT{
public:
    Tree tree;
    Tree tree2; // For the second robot in the two-robot case

    RRT(const Problem& problem);
    
    void addVertex(const Point& vertex, int parent_index); // Adds a vertex to the tree with the given parent index
    std::vector<Point> reconstructPath(int vertex_index) const; // Reconstructs the path from the root to the given vertex index
    Point randomSample_naive(const Problem& problem) const; // Samples a random point uniformly in the environment
    Point randomSample_intelligent(const Problem& problem, std::vector<Point> verticesObstacles, double p_vertex_obstacle, std::vector<Point> pointsNearObstacles, double p_edge_obstacle) const; // Samples a random point with intelligent method proposed in question 21
    bool edgeCollisionPath(const Problem& problem, const Point& p1, const double cost1, const Point& p2, const std::vector<Point>& path) const; // Checks if the edge between p1 and p2 intersects with any segment of the path
    int buildRRT(const Problem& problem, double delta_s, double delta_r, int max_iterations, bool use_intelligent_sampling=false, double p_vertex_obstacle=0.2, double p_edge_obstacle=0.3, int num_points_near_obstacles=1000, bool is_second_robot=false, std::vector<Point> path_first_robot={}); // Builds the RRT, returns the number of iterations taken to build the tree
    std::tuple<std::vector<Point>, int, double> rrtPath(const Problem& problem, double delta_s, double delta_r, int max_iterations, bool use_intelligent_sampling=false, double p_vertex_obstacle=0.2, double p_edge_obstacle=0.3, int num_points_near_obstacles=1000, bool is_second_robot=false, std::vector<Point> path_first_robot={}); // Builds the RRT and returns the path from start to goal, the number of iterations taken, and the cost of the path
    std::tuple<std::vector<Point>, double> optimizePath(const Problem& problem, std::vector<Point> path); // Optimizes the given path by removing unnecessary intermediate nodes, returns the optimized path and its cost
    std::tuple<std::vector<Point>, std::vector<Point>> rrtPath2Robots(const Problem& problem, double delta_s, double delta_r, int max_iterations, bool use_intelligent_sampling=false, double p_vertex_obstacle=0.2, double p_edge_obstacle=0.3, int num_points_near_obstacles=1000); // Builds the RRT for two robots and returns the paths for both robots
};
    