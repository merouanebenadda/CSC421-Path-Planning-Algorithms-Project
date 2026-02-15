#include <vector>
#include <utility>
#include <math.h>
#include <functional>
#include <algorithm>
#include <tuple>

#include "RRT.hpp"
#include "Problem.hpp"
#include "utils.hpp"

Tree::Tree(Point root) {
    // Initialize the tree with the given root point
    vertices.push_back(root);
    parents.push_back(-1); // Root has no parent
    costs.push_back(0.0); // Cost from root to itself is 0
}

RRT::RRT(const Problem& problem) : tree(problem.start1), tree2(problem.start2) {
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

bool RRT::edgeCollisionPath(const Problem& problem, const Point& p1, const double cost1, const Point& p2, const std::vector<Point>& path) const {
    // Check if the edge between p1 and p2 intersects with any segment of the path
    double cost_path = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        if (segmentsIntersect(p1, p2, path[i], path[i + 1])) {
            Point intersection_point;
            getIntersectionPoint(p1, p2, path[i], path[i + 1], intersection_point);
            if (std::abs(euclideanDistance(p1, intersection_point) + cost1 - (euclideanDistance(path[i],intersection_point) + cost_path)) < 2*problem.radius) {
                return true; // Collision detected
            }
        }
        cost_path += euclideanDistance(path[i], path[i+1]);
    }
    return false; // No collision
}

int RRT::buildRRT(const Problem& problem, double delta_s, double delta_r, int max_iterations, bool use_intelligent_sampling, double p_vertex_obstacle, double p_edge_obstacle, int num_points_near_obstacles, bool is_second_robot, std::vector<Point> path_first_robot) {
    // Implementation of the RRT algorithm to build the tree
    Tree& tree_cur = is_second_robot ? tree2 : tree; // Considered tree (tree or tree2 depending on the robot)
    std::vector<Point> verticesObstacles;
    std::vector<Point> pointsNearObstacles;
    if(use_intelligent_sampling) {
        verticesObstacles = problem.verticesObstacles();
        pointsNearObstacles = problem.pointsNearObstacles(num_points_near_obstacles); 
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
        for (size_t i = 1; i < tree_cur.vertices.size(); i++) {
            if (euclideanDistance(tree_cur.vertices[i], vr) < euclideanDistance(tree_cur.vertices[vn_index], vr)) {
                vn_index = i;
            }
        }

        // Create node v in the direction of vr at maximum distance delta_s from vn
        Point vn = tree_cur.vertices[vn_index];
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
        if (!problem.isCollision(vn, v) && !(is_second_robot && edgeCollisionPath(problem, vn, tree_cur.costs[vn_index], vr, path_first_robot))) {
            parent_index = vn_index;
        }
        for (size_t i = 0; i < tree_cur.vertices.size(); i++) {
            if (euclideanDistance(tree_cur.vertices[i], v) < delta_r 
                && !problem.isCollision(tree_cur.vertices[i], v)
                && !(is_second_robot && edgeCollisionPath(problem, tree_cur.vertices[i], tree_cur.costs[i], v, path_first_robot))
                && (parent_index == -1 
                    || tree_cur.costs[i] + euclideanDistance(tree_cur.vertices[i], v) < tree_cur.costs[parent_index] + euclideanDistance(tree_cur.vertices[parent_index], v))) {
                parent_index = i;
            }
        }
        if (parent_index == -1) {
            continue; // No valid parent found, skip this vertex
        }

        addVertex(v, parent_index);
        int index_v = tree_cur.vertices.size() - 1;
    
        // Update neighors' parent if it improves their cost
        if(!is_second_robot){
            for (size_t i = 0; i < tree_cur.vertices.size(); i++) {
                if (euclideanDistance(tree_cur.vertices[i], v) < delta_r 
                    && !problem.isCollision(tree_cur.vertices[i], v)
                    && tree_cur.costs[i] > tree_cur.costs[index_v] + euclideanDistance(tree_cur.vertices[index_v], tree_cur.vertices[i])) {
                    tree_cur.parents[i] = index_v; // Update parent to the new vertex
                    tree_cur.costs[i] = tree_cur.costs[index_v] + euclideanDistance(tree_cur.vertices[index_v], tree_cur.vertices[i]);
                }
            }
        }

        // Check if we can connect to the goal
        Point goal = is_second_robot ? problem.goal2 : problem.goal1;
        if (euclideanDistance(v, goal) <= delta_s && !problem.isCollision(v, goal)) {
            addVertex(goal, index_v);
            break; // Goal reached, exit the loop
        }

        iterations++;
    }

    return iterations;
}

std::tuple<std::vector<Point>, int, double> RRT::rrtPath(const Problem& problem, double delta_s, double delta_r, int max_iterations, bool use_intelligent_sampling, double p_vertex_obstacle, double p_edge_obstacle, int num_points_near_obstacles, bool is_second_robot, std::vector<Point> path_first_robot) {
    int iterations = buildRRT(problem, delta_s, delta_r, max_iterations, use_intelligent_sampling, p_vertex_obstacle, p_edge_obstacle, num_points_near_obstacles, is_second_robot, path_first_robot); 
    double path_cost = tree.costs.back(); // Cost of the path to the goal (last vertex added)
    if(is_second_robot) {
        path_cost = tree2.costs.back();
        return std::make_tuple(reconstructPath(tree2.vertices.size() - 1), iterations, path_cost); // The goal point is the last vertex added to the tree
    }else {
        return std::make_tuple(reconstructPath(tree.vertices.size() - 1), iterations, path_cost); // The goal point is the last vertex added to the tree
    }
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

std::tuple<std::vector<Point>, std::vector<Point>> RRT::rrtPath2Robots(const Problem& problem, double delta_s, double delta_r, int max_iterations, bool use_intelligent_sampling, double p_vertex_obstacle, double p_edge_obstacle, int num_points_near_obstacles) {
    // Build the RRT for the first robot and get its path
    auto [path_1, iterations_1, cost_1] = rrtPath(problem, delta_s, delta_r, max_iterations, use_intelligent_sampling, p_vertex_obstacle, p_edge_obstacle, num_points_near_obstacles);
    // Build the RRT for the second robot with the path of the first robot as additional obstacles
    auto [path_2, iterations_2, cost_2] = rrtPath(problem, delta_s, delta_r, max_iterations, use_intelligent_sampling, p_vertex_obstacle, p_edge_obstacle, num_points_near_obstacles, true, path_1); 
    return std::make_tuple(path_1, path_2);
}