#include <iostream>
#include <fstream> // for file handling
#include <string>
#include <random>

#include "Problem.hpp"
#include "PSO.hpp"

using namespace std;

// Function to test the PSO implementation
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
    PSO pso(problem, 100, 5); // Optionally, you can specify num_particles and num_waypoints here
    auto [best_path, best_cost] = pso.optimize(problem, 100000); // Optimize for 10000 iterations

    // Output results
    cout << "Best path found:" << endl;
    for (const auto& point : best_path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }

    cout << "Best cost: " << best_cost << endl;

    // Save results to a file for visualization
    string outputFileName = "output/paths/best_path" + to_string(time(0)) + ".txt";
    ofstream outputFile(outputFileName);
    if (outputFile.is_open()) {
        for (const auto& point : best_path) {
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

    return 0;
}

int main(int argc, char* argv[]) {
    return test_pso(argc, argv);
}