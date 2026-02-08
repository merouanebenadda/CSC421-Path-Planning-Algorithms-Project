#pragma once
#include "Problem.hpp"
#include <vector>
#include <utility>
#include <random>

// Hyperparameters for PSO
const int NUM_WAYPOINTS = 10;
const int NUM_PARTICLES = 30; 
const double C1 = 1.0; // acceleration coefficient for local best
const double C2 = 1.0; // acceleration coefficient for global best
const double W = 0.5;  // inertia weight for velocity


struct Particle{
    std::vector<Point> waypoints;
    std::vector<Point> velocity; 
    std::vector<Point> best_waypoints;
    double best_cost;

    Particle(const Problem& problem, int num_waypoints = NUM_WAYPOINTS); 
};

class PSO{
public:
    std::vector<Particle> particles;
    std::vector<Point> global_best_waypoints;
    double global_best_cost;

    PSO(const Problem& problem, int num_particles = NUM_PARTICLES, int num_waypoints = NUM_WAYPOINTS);

    std::pair<std::vector<Point>, double> optimize(const Problem& problem, int num_iterations,
    double c1 = C1, double c2 = C2, double w = W);
};

double fitness(const std::vector<Point>& waypoints, const Problem& problem);