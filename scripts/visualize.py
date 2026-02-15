"""
Script to visualize the environment, obstacles, start and goal points, and the path found by the algorithm.

Usage:
    python visualize.py scenario.txt --path path.txt
"""

import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plot_path_and_tree(ax, path_nums, start, goal, tree, path_label, path_color, tree_color):
    """Helper function to plot a path and its tree"""
    if len(path_nums) % 2 != 0:
        print(f"Error: Invalid path format for {path_label}.")
        return
    
    # Create path points and plot the path
    path_points = [start] + [(path_nums[i], path_nums[i+1]) for i in range(0, len(path_nums), 2)] + [goal]
    path_x, path_y = zip(*path_points)
    ax.plot(path_x, path_y, color=path_color, linewidth=2, label=path_label)

    # Plot the tree structure if available
    if tree:
        tree_vertices = list(tree.keys())  # Maintain order for indexing
        
        # Plot tree edges
        for vertex, parents in tree.items():
            for parent_index in parents:
                if parent_index >= 0:  # Skip root nodes (parent_index == -1)
                    parent_vertex = tree_vertices[parent_index]
                    ax.plot([vertex[0], parent_vertex[0]], [vertex[1], parent_vertex[1]], 
                            color=tree_color, alpha=0.2, linewidth=0.5)
        
        # Plot tree vertices as markers
        tree_x = [v[0] for v in tree_vertices]
        tree_y = [v[1] for v in tree_vertices]
        ax.plot(tree_x, tree_y, marker='x', color=tree_color, markersize=2, alpha=0.4, linestyle='None')

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
    fig, ax = plt.subplots(figsize=(10, 10))
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
    ax.plot(start1[0], start1[1], 'go', markersize=8, label='Start 1')
    ax.plot(goal1[0], goal1[1], 'ro', markersize=8, label='Goal 1')
    ax.plot(start2[0], start2[1], 'g^', markersize=8, label='Start 2')
    ax.plot(goal2[0], goal2[1], 'r^', markersize=8, label='Goal 2')

    # Plot path if provided
    if path_file:
        with open(path_file, 'r') as f:
            lines = f.readlines()
        
        # Check for two-path format (PATH1, PATH2, TREE1, TREE2)
        has_two_paths = any("PATH1\n" in line or line.strip() == "PATH1" for line in lines)
        
        if has_two_paths:
            # Parse two-path format
            path1_nums = []
            path2_nums = []
            tree1 = {}
            tree2 = {}
            
            current_section = None
            
            for line in lines:
                line = line.strip()
                if line == "PATH1":
                    current_section = "PATH1"
                elif line == "PATH2":
                    current_section = "PATH2"
                elif line == "TREE1":
                    current_section = "TREE1"
                elif line == "TREE2":
                    current_section = "TREE2"
                elif line and not line.startswith("#"):  # Skip empty lines and comments
                    if current_section == "PATH1":
                        path1_nums.extend(map(float, line.split()))
                    elif current_section == "PATH2":
                        path2_nums.extend(map(float, line.split()))
                    elif current_section == "TREE1":
                        parts = line.split()
                        if len(parts) >= 3:
                            x, y = float(parts[0]), float(parts[1])
                            parent = int(parts[2])
                            tree1[(x, y)] = [parent]
                    elif current_section == "TREE2":
                        parts = line.split()
                        if len(parts) >= 3:
                            x, y = float(parts[0]), float(parts[1])
                            parent = int(parts[2])
                            tree2[(x, y)] = [parent]
            
            # Plot both paths with different colors
            plot_path_and_tree(ax, path1_nums, start1, goal1, tree1, 'Path 1 (RRT)', 'blue', 'blue')
            plot_path_and_tree(ax, path2_nums, start1, goal1, tree2, 'Path 2 (RRT)', 'green', 'green')
        else:
            # Parse single-path format (original format)
            if "TREE\n" in lines or any(line.strip() == "TREE" for line in lines):
                tree_index = next(i for i, line in enumerate(lines) if line.strip() == "TREE")
                path_lines = lines[:tree_index]
                tree_lines = lines[tree_index + 1:]
            else:
                path_lines = lines
                tree_lines = []
            
            # Process path points
            path_nums = []
            for line in path_lines:
                path_nums.extend(map(float, line.split()))
            
            # Process tree data if available
            tree = {}
            for line in tree_lines:
                parts = line.split()
                if len(parts) >= 3:
                    x, y = float(parts[0]), float(parts[1])
                    parent = int(parts[2])
                    tree[(x, y)] = [parent]
            
            # Plot single path
            plot_path_and_tree(ax, path_nums, start1, goal1, tree, 'Path (RRT)', 'blue', 'blue')

    ax.legend(loc='best')
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