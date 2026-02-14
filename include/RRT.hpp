#pragma once

#include <vector>
#include <string>


struct Tree{
    std::vector<Point> vertices;
    std::vector<int> parents; // parents[i] gives the index of the parent of vertices[i]
    std::vector<double> costs; // costs[i] gives the cost from the root to vertices[i]
};

class RRT{
public:
    Tree tree;

    RRT(const Problem& problem);
    std::vector<Point> reconstructPath(int vertex_index) const; // Reconstructs the path from the root to the given vertex index
    void buildRRT(const Problem& problem, double delta_s, double delta_r, int max_iterations); // Builds the RRT

};
    