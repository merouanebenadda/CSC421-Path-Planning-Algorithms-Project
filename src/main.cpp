#include <iostream>
#include <fstream> // for file handling
#include <string>
#include <random>
#include <ctime>

#include "Problem.hpp"
#include "PSO.hpp"
#include "RRT.hpp"

using namespace std;

/// Hyperparameters for PSO

const int NUM_PARTICLES = 500;
const int NUM_WAYPOINTS = 5;
const int NUM_ITERATIONS = 30000;
const double C1 = 2.0; // cognitive coefficient
const double C2 = 2.0; // social coefficient
const double W = 0.75;  // inertia weight

// Random restart parameters
const int RESTART_INTERVAL = 5000; // Number of iterations after which to perform a random restart

// Annealing parameters
double initial_temperature = 100.0; // The larger, the more likely to accept worse solutions at the start
double cooling_rate = 0.99; // Between 0 and 1

// Dimensional learning parameters
int stagnation_threshold = 15; // Number of iterations without improvement before applying dimensional learning

// Fitness function choice
std::function<double(const std::vector<Point>&, const Problem&)> fitness_function = fitness_refined;


/// Hyperparameters for RRT

const double RRT_DELTA_S = 100.0; // Step size for extending the tree
const double RRT_DELTA_R = 100.0; // Radius for checking nearby vertices
const int RRT_MAX_ITERATIONS = 10000; // Maximum number of iterations to build the RRT

// Intelligent sampling parameters
const bool INTELLIGENT_SAMPLING = true; // Whether to use intelligent sampling  
const double P_VERTEX_OBSTACLE = 0.4; // Probability of sampling from obstacle vertices
const double P_EDGE_OBSTACLE = 0.3; // Probability of sampling from points
const int NUM_POINTS_NEAR_OBSTACLES = 1000; // Number of points to sample near obstacles for intelligent sampling

/*
@brief saves the given path and tree to a file and optionally visualizes them using a Python script if --plot flag is provided.
@param argc the number of command-line arguments
@param argv the array of command-line arguments
@param path the vector of Points representing the path to be saved and visualized
*/
void visualize(int argc, char* argv[], vector<Point> path, Tree* tree = nullptr) {
    // Save results to a file for visualization
    string outputFileName = "output/paths/best_path" + to_string(time(0)) + ".txt";
    ofstream outputFile(outputFileName);
    if (outputFile.is_open()) {
        for (const auto& point : path) {
            outputFile << point.x << " " << point.y << endl;
        }
        if (tree) {
            outputFile << "TREE" << endl; // Marker to indicate tree data follows
            for (size_t i = 0; i < tree->vertices.size(); i++) { // We use -> to access members of the Tree struct since it's passed as a pointer
                const auto& vertex = tree->vertices[i];
                outputFile << vertex.x << " " << vertex.y << " " << tree->parents[i] << endl;
            }
        }
        outputFile.close();
        cout << "Best path saved to " << outputFileName << endl;
    } else {
        cerr << "Error: Could not open file to save best path" << endl;
    }

    // Optional: Visualization
    if (argc == 3 && string(argv[2]) == "--plot") {
        // Call the visualization script
        int result = system(("python3 scripts/visualize.py " + string(argv[1])
         + " --path " + outputFileName).c_str()); // c_str() converts the string to a C-style string for system()
        if (result != 0) cerr << "Visualizer failed to launch." << endl;
    }
}

/*
@brief saves two paths and their corresponding trees to a file for comparison visualization.
@param argc the number of command-line arguments
@param argv the array of command-line arguments
@param path1 the vector of Points for the first path
@param tree1 pointer to the first tree
@param path2 the vector of Points for the second path
@param tree2 pointer to the second tree
*/
void visualize_two_paths(int argc, char* argv[], vector<Point> path1, Tree* tree1, vector<Point> path2, Tree* tree2) {
    // Save results to a file for visualization
    string outputFileName = "output/paths/best_path" + to_string(time(0)) + ".txt";
    ofstream outputFile(outputFileName);
    if (outputFile.is_open()) {
        // Write first path
        outputFile << "PATH1" << endl;
        for (const auto& point : path1) {
            outputFile << point.x << " " << point.y << endl;
        }
        
        // Write second path
        outputFile << "PATH2" << endl;
        for (const auto& point : path2) {
            outputFile << point.x << " " << point.y << endl;
        }
        
        // Write first tree
        if (tree1) {
            outputFile << "TREE1" << endl;
            for (size_t i = 0; i < tree1->vertices.size(); i++) {
                const auto& vertex = tree1->vertices[i];
                outputFile << vertex.x << " " << vertex.y << " " << tree1->parents[i] << endl;
            }
        }
        
        // Write second tree
        if (tree2) {
            outputFile << "TREE2" << endl;
            for (size_t i = 0; i < tree2->vertices.size(); i++) {
                const auto& vertex = tree2->vertices[i];
                outputFile << vertex.x << " " << vertex.y << " " << tree2->parents[i] << endl;
            }
        }
        
        outputFile.close();
        cout << "Two paths saved to " << outputFileName << endl;
    } else {
        cerr << "Error: Could not open file to save paths" << endl;
    }

    // Optional: Visualization
    if (argc == 3 && string(argv[2]) == "--plot") {
        // Call the visualization script with two paths
        int result = system(("python3 scripts/visualize.py " + string(argv[1])
         + " --path " + outputFileName).c_str());
        if (result != 0) cerr << "Visualizer failed to launch." << endl;
    }
}

// Test functions

// Functions to test the PSO implementations
int test_pso(int argc, char* argv[]){
    srand(time(0)); // Seed the random number generator

    if (argc < 2 || argc > 3) {
        cerr << "Usage: " << argv[0] << " <scenario_file> [--plot]" << endl;
        return 1;
    }

    // Load problem scenario
    Problem problem;
    if (!problem.loadScenario(argv[1])) {
        cerr << "Failed to load scenario from file: " << argv[1] << endl;
        return 1;
    }

    // PSO optimization
    PSO pso(problem, NUM_PARTICLES, NUM_WAYPOINTS); // Optionally, you can specify num_particles and num_waypoints here
    clock_t start_time = clock();
    auto [best_path, best_cost] = pso.optimize(problem, NUM_ITERATIONS, C1, C2, W, fitness_function); // Optimize for 10000 iterations
    clock_t end_time = clock();
    double cpu_time = double(end_time - start_time) / CLOCKS_PER_SEC;

    // Output results
    cout << "Best path found:" << endl;
    for (const auto& point : best_path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }

    cout << "Best cost: " << best_cost << endl;
    cout << "CPU time: " << cpu_time << " seconds" << endl;


    visualize(argc, argv, best_path);
    return 0;
}

int test_random_restart_pso(int argc, char* argv[]){
    srand(time(0)); // Seed the random number generator

    if (argc < 2 || argc > 3) {
        cerr << "Usage: " << argv[0] << " <scenario_file> [--plot]" << endl;
        return 1;
    }

    // Load problem scenario
    Problem problem;
    if (!problem.loadScenario(argv[1])) {
        cerr << "Failed to load scenario from file: " << argv[1] << endl;
        return 1;
    }

    // PSO optimization with random restarts
    PSO pso(problem, NUM_PARTICLES, NUM_WAYPOINTS); // Optionally, you can specify num_particles and num_waypoints here
    clock_t start_time = clock();
    auto [best_path, best_cost] = pso.optimize_with_random_restart(problem, NUM_ITERATIONS, C1, C2, W, 
        RESTART_INTERVAL, fitness_function); // Optimize with random restarts
    clock_t end_time = clock();
    double cpu_time = double(end_time - start_time) / CLOCKS_PER_SEC;

    // Output results
    cout << "Best path found:" << endl;
    for (const auto& point : best_path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }

    cout << "Best cost: " << best_cost << endl;
    cout << "CPU time: " << cpu_time << " seconds" << endl;

    visualize(argc, argv, best_path);
    return 0;
}

int test_annealing_pso(int argc, char* argv[]){
    srand(time(0)); // Seed the random number generator

    if (argc < 2 || argc > 3) {
        cerr << "Usage: " << argv[0] << " <scenario_file> [--plot]" << endl;
        return 1;
    }

    // Load problem scenario
    Problem problem;
    if (!problem.loadScenario(argv[1])) {
        cerr << "Failed to load scenario from file: " << argv[1] << endl;
        return 1;
    }

    // PSO optimization with annealing
    PSO pso(problem, NUM_PARTICLES, NUM_WAYPOINTS); // Optionally, you can specify num_particles and num_waypoints here
    clock_t start_time = clock();
    auto [best_path, best_cost] = pso.optimize_with_annealing(problem, NUM_ITERATIONS, C1, C2, W, RESTART_INTERVAL, 
        initial_temperature, cooling_rate, fitness_function); // Optimize with annealing
    clock_t end_time = clock();
    double cpu_time = double(end_time - start_time) / CLOCKS_PER_SEC;

    // Output results
    cout << "Best path found:" << endl;
    for (const auto& point : best_path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }

    cout << "Best cost: " << best_cost << endl;
    cout << "CPU time: " << cpu_time << " seconds" << endl;

    visualize(argc, argv, best_path);
    return 0;
}

int test_dimensional_learning_pso(int argc, char* argv[]){
    srand(time(0)); // Seed the random number generator

    if (argc < 2 || argc > 3) {
        cerr << "Usage: " << argv[0] << " <scenario_file> [--plot]" << endl;
        return 1;
    }

    // Load problem scenario
    Problem problem;
    if (!problem.loadScenario(argv[1])) {
        cerr << "Failed to load scenario from file: " << argv[1] << endl;
        return 1;
    }

    // PSO optimization with dimensional learning
    PSO pso(problem, NUM_PARTICLES, NUM_WAYPOINTS); // Optionally, you can specify num_particles and num_waypoints here
    clock_t start_time = clock();
    auto [best_path, best_cost] = pso.optimize_with_dimensional_learning(problem, NUM_ITERATIONS, C1, C2, W, 
        RESTART_INTERVAL, initial_temperature, cooling_rate, stagnation_threshold, fitness_function); // Optimize with dimensional learning
    clock_t end_time = clock();
    double cpu_time = double(end_time - start_time) / CLOCKS_PER_SEC;

    // Output results
    cout << "Best path found:" << endl;
    for (const auto& point : best_path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }

    cout << "Best cost: " << best_cost << endl;
    cout << "CPU time: " << cpu_time << " seconds" << endl;

    visualize(argc, argv, best_path);
    return 0;
}

void write_path(std::ostream& out, const std::vector<Point>& path) {
    for (const auto& point : path) {
        out << "(" << point.x << ", " << point.y << ")" << std::endl;
    }
}

int test_rrt(int argc, char* argv[]){
    srand(time(0)); // Seed the random number generator

    if (argc < 2 || argc > 3) {
        cerr << "Usage: " << argv[0] << " <scenario_file> [--plot]" << endl;
        return 1;
    }

    // Load problem scenario
    Problem problem;
    if (!problem.loadScenario(argv[1])) {
        cerr << "Failed to load scenario from file: " << argv[1] << endl;
        return 1;
    }

    // RRT
    RRT rrt(problem); 
    clock_t start_time = clock();
    auto [best_path, iterations, path_cost] = rrt.rrtPath(problem, RRT_DELTA_S, RRT_DELTA_R, RRT_MAX_ITERATIONS, INTELLIGENT_SAMPLING, P_VERTEX_OBSTACLE, P_EDGE_OBSTACLE, NUM_POINTS_NEAR_OBSTACLES);
    clock_t end_time = clock();
    double cpu_time = double(end_time - start_time) / CLOCKS_PER_SEC;

    // Output results
    cout << "Best path found:" << endl;
    for (const auto& point : best_path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }

    cout << "CPU time: " << cpu_time << " seconds" << endl;
    cout << "Iterations: " << iterations << endl;
    cout << "Path cost: " << path_cost << endl;



    visualize(argc, argv, best_path, &rrt.tree);
    return 0;
}

int test_rrt_optimized(int argc, char* argv[]){
    srand(time(0)); // Seed the random number generator

    if (argc < 2 || argc > 3) {
        cerr << "Usage: " << argv[0] << " <scenario_file> [--plot]" << endl;
        return 1;
    }

    // Load problem scenario
    Problem problem;
    if (!problem.loadScenario(argv[1])) {
        cerr << "Failed to load scenario from file: " << argv[1] << endl;
        return 1;
    }

    // RRT optimization
    RRT rrt(problem); 
    clock_t start_time = clock();
    auto [initial_path, iterations, initial_cost] = rrt.rrtPath(problem, RRT_DELTA_S, RRT_DELTA_R, RRT_MAX_ITERATIONS);
    
    clock_t end_build_time = clock();
    auto [optimized_path, optimized_cost] = rrt.optimizePath(problem, initial_path);
    clock_t end_optimize_time = clock();
    double cpu_time_build = double(end_build_time - start_time) / CLOCKS_PER_SEC;
    double cpu_time_optimize = double(end_optimize_time - end_build_time) / CLOCKS_PER_SEC;

    // Output results
    cout << "Initial path found:" << endl;
    for (const auto& point : initial_path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }
    cout << "Initial path cost: " << initial_cost << endl;

    cout << "\nOptimized path:" << endl;
    for (const auto& point : optimized_path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }
    cout << "Optimized path cost: " << optimized_cost << endl;

    cout << "\nCPU time for building RRT: " << cpu_time_build << " seconds" << endl;
    cout << "\nCPU time for optimization: " << cpu_time_optimize << " seconds" << endl;
    cout << "Iterations: " << iterations << endl;

    visualize(argc, argv, optimized_path, &rrt.tree);
    return 0;
}


int main(int argc, char* argv[]) {
    return test_dimensional_learning_pso(argc, argv);
    //return test_rrt(argc, argv);
    //return test_rrt_optimized(argc, argv);
    //return test_two_rrt_paths(argc, argv);
}