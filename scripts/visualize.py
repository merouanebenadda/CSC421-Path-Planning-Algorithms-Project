"""
Script to visualize the environment, obstacles, start and goal points, and the path found by the algorithm.

Usage:
    python visualize.py scenario.txt --path path.txt
"""

import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def visualize(scenario_file, path_file=None):
    with open(scenario_file, 'r') as f:
        nums = list(map(float, f.read().split()))

    if len(nums) < 11:
        print("Error: Invalid scenario file format.")
        return
    
    # Extract scenario parameters
    xmax, ymax = nums[0], nums[1]
    start1 = (nums[2], nums[3])
    goal1 = (nums[4], nums[5])
    start2 = (nums[6], nums[7])
    goal2 = (nums[8], nums[9])
    radius = nums[10]

    obs_data = nums[11:]
    obstacles = [obs_data[i:i+4] for i in range(0, len(obs_data), 4)]

    # Create figure and axis
    fig, ax = plt.subplots()
    ax.set_xlim(0, xmax)
    ax.set_ylim(0, ymax)
    ax.set_aspect('equal')

    # Plot obstacles
    for obs in obstacles:
        ll_corner = (obs[0], obs[1])
        width, height = obs[2], obs[3]
        rect = patches.Rectangle(ll_corner, width, height, linewidth=1, edgecolor='black', facecolor='gray')
        ax.add_patch(rect)

    # Plot start and goal points
    ax.plot(start1[0], start1[1], 'go', label='Start 1')
    ax.plot(goal1[0], goal1[1], 'ro', label='Goal 1')
    ax.plot(start2[0], start2[1], 'gx', label='Start 2')
    ax.plot(goal2[0], goal2[1], 'rx', label='Goal 2')

    # Plot path if provided
    if path_file:
        with open(path_file, 'r') as f:
            path_nums = list(map(float, f.read().split()))
        
        if len(path_nums) % 2 != 0:
            print("Error: Invalid path file format.")
            return
        
        path_points = [(path_nums[i], path_nums[i+1]) for i in range(0, len(path_nums), 2)]
        path_x, path_y = zip(*path_points)
        ax.plot(path_x, path_y, 'b-', label='Path')

    ax.legend()
    plt.title("Environment Visualization")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.grid()
    plt.tight_layout()
    plt.show()


def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Visualize the environment and path.")
    parser.add_argument("scenario", help="Path to the problem file.")
    parser.add_argument("--path", help="Path to the file containing the path", default=None)
    args = parser.parse_args()

    print(f"Loading scenario: {args.scenario}")
    if args.path:
        print(f"Overlaying path: {args.path}")

    visualize(args.scenario, args.path)


if __name__ == "__main__":
    main()