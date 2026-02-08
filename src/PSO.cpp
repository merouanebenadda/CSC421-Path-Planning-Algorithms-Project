#include <vector>
#include <utility>

#include "PSO.hpp"
#include "Problem.hpp"
#include "utils.hpp"

// CONSTANTS
const double INF = 1e9;

Particle::Particle(const Problem& problem, int num_waypoints) : best_cost(INF) {
    // Initialize waypoints randomly within the environment bounds
    for (int i = 0; i < num_waypoints; ++i) {
        double x = static_cast<double>(rand()) / RAND_MAX * problem.x_max;
        double y = static_cast<double>(rand()) / RAND_MAX * problem.y_max;
        waypoints.emplace_back(x, y);
        velocity.emplace_back(0.0, 0.0); // Start with zero velocity
    }
    // Initialize best_waypoints to current waypoints
    best_waypoints = waypoints;
}

PSO::PSO(const Problem& problem, int num_particles, int num_waypoints) : global_best_cost(INF) {
    // Initialize particles
    for (int i = 0; i < num_particles; ++i) {
        particles.emplace_back(problem, num_waypoints); // emplace_back constructs a Particle in place using its constructor
    }
    // Initialize global_best_waypoints with the first particle's waypoints
    if (!particles.empty()) {
        global_best_waypoints = particles[0].waypoints;
    }
}

std::pair<std::vector<Point>, double> PSO::optimize(const Problem& problem, int num_iterations,
    double c1, double c2, double w) {
    // Ensure global_best_waypoints is initialized
    if (global_best_waypoints.empty() && !particles.empty()) {
        global_best_waypoints = particles[0].waypoints;
    }
    for (int iter = 0; iter < num_iterations; ++iter) {
        for (auto& particle : particles) {
            // Update particle's best known position
            double cost = fitness(particle.waypoints, problem);
            if (cost < particle.best_cost) {
                particle.best_cost = cost;
                particle.best_waypoints = particle.waypoints;
            }

            // Update global best position
            if (cost < global_best_cost) {
                global_best_cost = cost;
                global_best_waypoints = particle.waypoints;
            }
        }

        // Update velocities and positions of particles
        for (auto& particle : particles) {
            for (size_t i = 0; i < particle.waypoints.size(); ++i) {
                // Update velocity based on local and global bests
                double r1 = static_cast<double>(rand()) / RAND_MAX; // random in [0, 1]
                double r2 = static_cast<double>(rand()) / RAND_MAX; // random in [0, 1]

                particle.velocity[i].x = w * particle.velocity[i].x +
                                        c1 * r1 * (particle.best_waypoints[i].x - particle.waypoints[i].x) +
                                        c2 * r2 * (global_best_waypoints[i].x - particle.waypoints[i].x);

                particle.velocity[i].y = w * particle.velocity[i].y +
                                        c1 * r1 * (particle.best_waypoints[i].y - particle.waypoints[i].y) +
                                        c2 * r2 * (global_best_waypoints[i].y - particle.waypoints[i].y);

                // Update position
                particle.waypoints[i].x += particle.velocity[i].x;
                particle.waypoints[i].y += particle.velocity[i].y;

                // Ensure waypoints are within bounds of the environment
                particle.waypoints[i].x = std::max(0.0, std::min(particle.waypoints[i].x, problem.x_max));
                particle.waypoints[i].y = std::max(0.0, std::min(particle.waypoints[i].y, problem.y_max));
            }
        }
    }
    return {global_best_waypoints, global_best_cost};
}


/*
* @brief Objective function for the PSO problem, which should be minimized.
* @param waypoints The waypoints of the path to evaluate.
* @param problem The problem instance containing the environment and obstacles.
*/
double fitness(const std::vector<Point>& waypoints, const Problem& problem) {
    double total_distance = 0.0;
    Point current = problem.start1;

    // Check for collisions
    if (problem.isCollision(waypoints)) {
        return INF; // Penalize paths that collide with obstacles
    }

    // Calculate total path length
    for (const auto& wp : waypoints) {
        total_distance += euclideanDistance(current, wp);
        current = wp;
    }
    total_distance += euclideanDistance(current, problem.goal1);

    return total_distance;
}