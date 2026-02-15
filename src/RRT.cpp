#include <vector>
#include <utility>
#include <math.h>
#include <functional>
#include <algorithm>
#include <tuple>

#include "RRT.hpp"
#include "Problem.hpp"
#include "utils.hpp"

Tree::Tree(const Problem& problem) {
    // Initialize the tree with the start point as the root
    vertices.push_back(problem.start1);
    parents.push_back(-1); // Root has no parent
    costs.push_back(0.0); // Cost from root to itself is 0
}

RRT::RRT(const Problem& problem) : tree(problem) {
    // The constructor initializes the tree with the start point
}

void RRT::addVertex(const Point& vertex, int parent_index) {
    tree.vertices.push_back(vertex);
    tree.parents.push_back(parent_index);
    tree.costs.push_back(tree.costs[parent_index] + euclideanDistance(tree.vertices[parent_index], vertex));
}

std::vector<Point> RRT::reconstructPath(int vertex_index) const {
    // Reconstruct the path (first and last points excluded) from the root to the given vertex index
    std::vector<Point> path;
    vertex_index = tree.parents[vertex_index]; // Start from the parent of the goal vertex
    while (tree.parents[vertex_index] != -1) {
        path.push_back(tree.vertices[vertex_index]);
        vertex_index = tree.parents[vertex_index];
    }
    std::reverse(path.begin(), path.end()); // Reverse the path to get it from start to goal
    return path;
}

Point RRT::randomSample_naive(const Problem& problem) const {
    // Sample a random point uniformly in the environment
    double x = static_cast<double>(rand()) / RAND_MAX * problem.x_max;
    double y = static_cast<double>(rand()) / RAND_MAX * problem.y_max;
    return Point(x, y);
}

Point RRT::randomSample_intelligent(const Problem& problem, std::vector<Point> verticesObstacles, double p_vertex_obstacle, std::vector<Point> pointsNearObstacles, double p_edge_obstacle) const {
    // Sample a random point with intelligent method proposed in question 21
    double r = static_cast<double>(rand()) / RAND_MAX; // random in [0, 1]
    if (r < p_vertex_obstacle && !verticesObstacles.empty()) {
        return verticesObstacles[rand() % verticesObstacles.size()]; // Sample from obstacle vertices
    } else if (r < p_vertex_obstacle + p_edge_obstacle && !pointsNearObstacles.empty()) {
        return pointsNearObstacles[rand() % pointsNearObstacles.size()]; // Sample from points near obstacles
    } else {
        return randomSample_naive(problem); // Sample uniformly from the environment
    }
}

int RRT::buildRRT(const Problem& problem, double delta_s, double delta_r, int max_iterations, bool use_intelligent_sampling, double p_vertex_obstacle, double p_edge_obstacle, int num_points_near_obstacles) {
    // Implementation of the RRT algorithm to build the tree
    
    std::vector<Point> verticesObstacles;
    std::vector<Point> pointsNearObstacles;
    if(use_intelligent_sampling) {
        std::vector<Point> verticesObstacles = problem.verticesObstacles();
        std::vector<Point> pointsNearObstacles = problem.pointsNearObstacles(num_points_near_obstacles); 
    }
    
    int iterations = 0;
    while(iterations < max_iterations){
        
        Point vr;
        if(use_intelligent_sampling) {
            vr = randomSample_intelligent(problem, verticesObstacles, p_vertex_obstacle, pointsNearObstacles, p_edge_obstacle);
        } else {
            vr = randomSample_naive(problem);
        }

        if(pointInObstacles(vr, problem.obstacles)){
            continue; // Skip if the random point is inside an obstacle
        }
        // Find the nearest vertex in the tree
        int vn_index = 0;
        for (size_t i = 1; i < tree.vertices.size(); i++) {
            if (euclideanDistance(tree.vertices[i], vr) < euclideanDistance(tree.vertices[vn_index], vr)) {
                vn_index = i;
            }
        }

        // Create node v in the direction of vr at maximum distance delta_s from vn
        Point vn = tree.vertices[vn_index];
        Point v;
        double dist = euclideanDistance(vn, vr);
        if (dist <= delta_s) {
            v = vr;
        } else {
            double theta = atan2(vr.y - vn.y, vr.x - vn.x);
            v = Point(vn.x + delta_s * cos(theta), vn.y + delta_s * sin(theta));
        }
        // Choose the parent of v
        int parent_index = -1;
        if (!problem.isCollision(vn, v)) {
            parent_index = vn_index;
        }
        for (size_t i = 0; i < tree.vertices.size(); i++) {
            if (euclideanDistance(tree.vertices[i], v) < delta_r 
                && !problem.isCollision(tree.vertices[i], v)
                && (parent_index == -1 
                    || tree.costs[i] + euclideanDistance(tree.vertices[i], v) < tree.costs[parent_index] + euclideanDistance(tree.vertices[parent_index], v))) {
                parent_index = i;
            }
        }
        if (parent_index == -1) {
            continue; // No valid parent found, skip this vertex
        }

        addVertex(v, parent_index);
        int index_v = tree.vertices.size() - 1;
    
        // Update neighors' parent if it improves their cost
        for (size_t i = 0; i < tree.vertices.size(); i++) {
            if (euclideanDistance(tree.vertices[i], v) < delta_r 
                && !problem.isCollision(tree.vertices[i], v)
                && tree.costs[i] > tree.costs[index_v] + euclideanDistance(tree.vertices[index_v], tree.vertices[i])) {
                tree.parents[i] = index_v; // Update parent to the new vertex
                tree.costs[i] = tree.costs[index_v] + euclideanDistance(tree.vertices[index_v], tree.vertices[i]);
            }
        }

        // Check if we can connect to the goal
        if (euclideanDistance(v, problem.goal1) <= delta_s && !problem.isCollision(v, problem.goal1)) {
            addVertex(problem.goal1, index_v);
            break; // Goal reached, exit the loop
        }

        iterations++;
    }

    return iterations;
}

std::tuple<std::vector<Point>, int, double> RRT::rrtPath(const Problem& problem, double delta_s, double delta_r, int max_iterations, bool use_intelligent_sampling, double p_vertex_obstacle, double p_edge_obstacle, int num_points_near_obstacles) {
    int iterations =buildRRT(problem, delta_s, delta_r, max_iterations, use_intelligent_sampling, p_vertex_obstacle, p_edge_obstacle, num_points_near_obstacles); 
    double path_cost = tree.costs.back(); // Cost of the path to the goal (last vertex added)
    return std::make_tuple(reconstructPath(tree.vertices.size() - 1), iterations, path_cost); // The goal point is the last vertex added to the tree

}

std::tuple<std::vector<Point>, double> RRT::optimizePath(const Problem& problem, std::vector<Point> path){
    // Optimize the path by removing unnecessary intermediate nodes
    std::vector<Point> optimized_path;
    optimized_path.push_back(problem.start1); // Start point

    for (size_t i = 1; i < path.size(); i++) {
        if (i == path.size() - 1 || problem.isCollision(optimized_path.back(), path[i + 1])) {
            optimized_path.push_back(path[i]); // Add the last point or the point before a collision
        }
    }

    double optimized_cost = 0.0;
    for (size_t i = 0; i < optimized_path.size() - 1; i++) {
        optimized_cost += euclideanDistance(optimized_path[i], optimized_path[i + 1]);
    }

    return std::make_tuple(optimized_path, optimized_cost);
}

