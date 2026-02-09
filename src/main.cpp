#include <iostream>
#include <fstream> // for file handling
#include <string>
#include <random>

#include "Problem.hpp"
#include "PSO.hpp"

using namespace std;

/// Hyperparameters for PSO

// Base algorithm parameters
const int NUM_PARTICLES = 100;
const int NUM_WAYPOINTS = 8;
const int NUM_ITERATIONS = 1000000;
const double C1 = 1.0; // cognitive coefficient
const double C2 = 1.0; // social coefficient
const double W = 0.75;  // inertia weight

// Random restart parameters
const int RESTART_INTERVAL = 5000; // Number of iterations after which to perform a random restart

// Annealing parameters
double initial_temperature = 10.0; // The larger, the more likely to accept worse solutions at the start
double cooling_rate = 0.999; // Between 0 and 1

// Dimensional learning parameters
int stagnation_threshold = 100; // Number of iterations without improvement before applying dimensional learning

/*
@brief saves the given path to a file and optionally visualizes it using a Python script if --plot flag is provided.
@param argc the number of command-line arguments
@param argv the array of command-line arguments
@param path the vector of Points representing the path to be saved and visualized
*/
void visualize(int argc, char* argv[], vector<Point> path){
    // Save results to a file for visualization
    string outputFileName = "output/paths/best_path" + to_string(time(0)) + ".txt";
    ofstream outputFile(outputFileName);
    if (outputFile.is_open()) {
        for (const auto& point : path) {
            outputFile << point.x << " " << point.y << endl;
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
    auto [best_path, best_cost] = pso.optimize(problem, NUM_ITERATIONS, C1, C2, W); // Optimize for 10000 iterations

    // Output results
    cout << "Best path found:" << endl;
    for (const auto& point : best_path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }

    cout << "Best cost: " << best_cost << endl;


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
    auto [best_path, best_cost] = pso.optimize_with_random_restart(problem, NUM_ITERATIONS, C1, C2, W, RESTART_INTERVAL); // Optimize with random restarts

    // Output results
    cout << "Best path found:" << endl;
    for (const auto& point : best_path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }

    cout << "Best cost: " << best_cost << endl;

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
    auto [best_path, best_cost] = pso.optimize_with_annealing(problem, NUM_ITERATIONS, C1, C2, W, RESTART_INTERVAL, initial_temperature, cooling_rate); // Optimize with annealing

    // Output results
    cout << "Best path found:" << endl;
    for (const auto& point : best_path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }

    cout << "Best cost: " << best_cost << endl;

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
    auto [best_path, best_cost] = pso.optimize_with_dimensional_learning(problem, NUM_ITERATIONS, C1, C2, W, RESTART_INTERVAL, initial_temperature, cooling_rate, stagnation_threshold); // Optimize with dimensional learning

    // Output results
    cout << "Best path found:" << endl;
    for (const auto& point : best_path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }

    cout << "Best cost: " << best_cost << endl;

    visualize(argc, argv, best_path);
    return 0;
}

int main(int argc, char* argv[]) {
    return test_dimensional_learning_pso(argc, argv);
}