# CSC421 : PATH PLANNING ALGORITHMS 
## Merouane Benadda, Antoine Fèvre

A programming project for Polytechnique's CSC 421 Algorithms course.

### Guide for the grader

You can find the correspoding files for each question in the [`ANSWERS.md`](ANSWERS.md) file.
The report is available in the [`report.pdf`](report/report.pdf) file.

### How to run the code

First edit the main function to select the test method you want to run. Then, you can build and run the code using the following command (here for scenario 1, but you can replace it with any of the other scenarios in the `assets/scenarios` folder):

```bash

### Clean, build and run (--plot is optional, it will display the path and obstacles)
make clean && make && ./path_planner assets/scenarios/scenario1.txt --plot

```