#pragma once

#include <vector>
#include <utility>
#include <random>
#include <functional> // To pass the fitness function as a parameter

#include "Problem.hpp"


struct Particle{
    std::vector<Point> waypoints;
    std::vector<Point> velocity; 
    std::vector<Point> best_waypoints;
    double best_cost;
    int stagnation_counter;

    Particle(const Problem& problem, int num_waypoints); 
};

class PSO{
public:
    std::vector<Particle> particles;
    std::vector<Point> global_best_waypoints;
    double global_best_cost;

    PSO(const Problem& problem, int num_particles, int num_waypoints);

    std::pair<std::vector<Point>, double> optimize(const Problem& problem, int num_iterations,
    double c1, double c2, double w, std::function<double(const std::vector<Point>&, const Problem&)> fitness);

    std::pair<std::vector<Point>, double> optimize_with_random_restart(const Problem& problem, int num_iterations,
    double c1, double c2, double w, int restart_interval, std::function<double(const std::vector<Point>&, const Problem&)> fitness);

    std::pair<std::vector<Point>, double> optimize_with_annealing(const Problem& problem, int num_iterations,
    double c1, double c2, double w, int restart_interval, double initial_temp, double cooling_rate, std::function<double(const std::vector<Point>&, const Problem&)> fitness);

    std::pair<std::vector<Point>, double> optimize_with_dimensional_learning(const Problem& problem, int num_iterations,
    double c1, double c2, double w, int restart_interval, double initial_temp, double cooling_rate, int stagnation_threshold, std::function<double(const std::vector<Point>&, const Problem&)> fitness);
};

double fitness(const std::vector<Point>& waypoints, const Problem& problem);

double fitness_refined(const std::vector<Point>& waypoints, const Problem& problem);