#pragma once
#include "Problem.hpp"
#include <vector>
#include <utility>
#include <random>



struct Particle{
    std::vector<Point> waypoints;
    std::vector<Point> velocity; 
    std::vector<Point> best_waypoints;
    double best_cost;

    Particle(const Problem& problem, int num_waypoints); 
};

class PSO{
public:
    std::vector<Particle> particles;
    std::vector<Point> global_best_waypoints;
    double global_best_cost;

    PSO(const Problem& problem, int num_particles, int num_waypoints);

    std::pair<std::vector<Point>, double> optimize(const Problem& problem, int num_iterations,
    double c1, double c2, double w);

    std::pair<std::vector<Point>, double> optimize_with_random_restart(const Problem& problem, int num_iterations,
    double c1, double c2, double w, int restart_interval);

    std::pair<std::vector<Point>, double> optimize_with_annealing(const Problem& problem, int num_iterations,
    double c1, double c2, double w, int restart_interval, double initial_temp, double cooling_rate);
};

double fitness(const std::vector<Point>& waypoints, const Problem& problem);