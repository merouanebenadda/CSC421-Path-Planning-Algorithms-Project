#include <iostream>
#include <fstream> // for file handling
#include <string>

#include "Problem.hpp"

using namespace std;

int main(int argc, char* argv[]){
    // Check if the user provided a filename as an argument
    if (argc < 2) {
        cerr << "Error: No filename provided" << endl;
        return 1;
    }

    string filename = argv[1];

    // Open the file for reading
    ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        cerr << "Error: Could not open file " << filename << endl;
        return 1;
    }

    // Read the contents of the scenario file and check for errors
    Problem problem;
    if (!(inputFile >> problem.x_max >> problem.y_max >> problem.start1.x >> problem.start1.y 
                    >> problem.goal1.x >> problem.goal1.y 
                    >> problem.start2.x >> problem.start2.y 
                    >> problem.goal2.x >> problem.goal2.y 
                    >> problem.radius)) {
        cerr << "Error: Invalid file format" << endl;
        return 1;
    }

    // Check for valid dimensions and radius
    if (problem.x_max <= 0 || problem.y_max <= 0) {
        cerr << "Error: Invalid environment dimensions" << endl;
        return 1;
    }
    if (problem.radius < 0) {
        cerr << "Error: Invalid radius" << endl;
        return 1;
    }
    if (problem.start1.x < 0 || problem.start1.x > problem.x_max || problem.start1.y < 0 || problem.start1.y > problem.y_max ||
        problem.goal1.x < 0 || problem.goal1.x > problem.x_max || problem.goal1.y < 0 || problem.goal1.y > problem.y_max ||
        problem.start2.x < 0 || problem.start2.x > problem.x_max || problem.start2.y < 0 || problem.start2.y > problem.y_max ||
        problem.goal2.x < 0 || problem.goal2.x > problem.x_max || problem.goal2.y < 0 || problem.goal2.y > problem.y_max) {
        cerr << "Error: Start or goal positions are out of bounds" << endl;
        return 1;
    }


    double x, y, lx, ly;
    while (inputFile >> x >> y >> lx >> ly) {
        if (lx <= 0 || ly <= 0) {
            cerr << "Error: Invalid obstacle dimensions" << endl;
            return 1;
        }

        if (x < 0 || x > problem.x_max || y < 0 || y > problem.y_max) {
            cerr << "Error: Obstacle position is out of bounds" << endl;
            return 1;
        }

        if (x + lx > problem.x_max || y + ly > problem.y_max) {
            cerr << "Error: Obstacle exceeds environment bounds" << endl;
            return 1;
        }

        Obstacle obs;
        obs.ll_corner = Point(x, y); 
        obs.lx = lx;
        obs.ly = ly;
        problem.obstacles.push_back(obs);
    }

    if (!inputFile.eof()) {
        cerr << "Error: Invalid file format in obstacles" << endl;
        return 1;
    }


    return 0;
}