# Answers

## Implementation choices
We chose to implement the solution in C++ for the main algorithmic part, as it allows for better performance and control over memory management, which is crucial for optimization problems like PSO. For visualization, we opted for Python with Matplotlib, as it provides a more convenient way to create visual representations of the results.

## Answers to the questions

**Question 1** : ['src/Problem.cpp'](src/Problem.cpp) and ['include/Problem.hpp'](include/Problem.hpp) for the input extraction and the implementation of the problem class.

**Question 2** : ['script/visualize.py'](script/visualize.py) for the visualization of the results. We chose to use Python and Matplotlib for this part, as it is more convenient for plotting and visualizing data.

**Question 3** [src/PSO.cpp](src/PSO.cpp) and [include/PSO.hpp](include/PSO.hpp) for the definition of the Particle class and and the fitness function. 

**Question 4** [src/Problem.cpp](src/Problem.cpp) and [include/Problem.hpp](include/Problem.hpp) for the implementation of the collision checking methods. [src/utils.cpp](src/utils.cpp) for the geometric logic.

**Question 7** [src/PSO.cpp](src/PSO.cpp) and [include/PSO.hpp](include/PSO.hpp) for the implementation of the PSO algorithm.

**Question 13** [include/RRT.hpp](include/RRT.hpp) for the tree data structure. Parent pointer representation is used.

**Question 14** [src/RRT.cpp](src/RRT.cpp) and [include/RRT.hpp](include/RRT.hpp) for the path reconstruction method.

**Question 17** [src/RRT.cpp](src/RRT.cpp) and [include/RRT.hpp](include/RRT.hpp) for the implementation of the intelligent sampling method.

**Question 19** [src/RRT.cpp](src/RRT.cpp) and [include/RRT.hpp](include/RRT.hpp) for the implementation of the RRT algorithm.

**Question 22** [src/RRT.cpp](src/RRT.cpp) and [include/RRT.hpp](include/RRT.hpp) for the implementation of the RRT for two robots, including the method to check for edge collisions with the path of the first robot.

