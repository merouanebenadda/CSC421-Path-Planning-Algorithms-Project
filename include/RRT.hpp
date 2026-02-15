#pragma once

#include <vector>
#include <string>

#include "Problem.hpp"


struct Tree{
    std::vector<Point> vertices;
    std::vector<int> parents; // parents[i] gives the index of the parent of vertices[i]
    std::vector<double> costs; // costs[i] gives the cost from the root to vertices[i]

    Tree(const Problem& problem); // Initializes the tree with the start point
};

class RRT{
public:
    Tree tree;

    RRT(const Problem& problem);
    
    void addVertex(const Point& vertex, int parent_index); // Adds a vertex to the tree with the given parent index
    std::vector<Point> reconstructPath(int vertex_index) const; // Reconstructs the path from the root to the given vertex index
    int buildRRT(const Problem& problem, double delta_s, double delta_r, int max_iterations); // Builds the RRT, returns the number of iterations taken to build the tree
    std::tuple<std::vector<Point>, int, double> rrtPath(const Problem& problem, double delta_s, double delta_r, int max_iterations); // Builds the RRT and returns the path from start to goal, the number of iterations taken, and the cost of the path

};
    