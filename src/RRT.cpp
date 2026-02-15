#include <vector>
#include <utility>
#include <math.h>
#include <functional>
#include <algorithm>

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

void RRT::buildRRT(const Problem& problem, double delta_s, double delta_r, int max_iterations) {
    // Implementation of the RRT algorithm to build the tree
    int iterations = 0;
    while(iterations < max_iterations){
        double x = static_cast<double>(rand()) / RAND_MAX * problem.x_max;
        double y = static_cast<double>(rand()) / RAND_MAX * problem.y_max;
        Point vr = Point(x, y);

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
}

std::vector<Point> RRT::rrtPath(const Problem& problem, double delta_s, double delta_r, int max_iterations) {
    buildRRT(problem, delta_s, delta_r, max_iterations); 
    return reconstructPath(tree.vertices.size() - 1); // The goal point is the last vertex added to the tree

}

