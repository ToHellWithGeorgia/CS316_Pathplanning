#include <stdio.h>
#include <stdlib.h>
#include "RRT.h"
#include "PathPlanning.h"

int main()
{
  struct pathPlanning rrt;
  rrt._start_state = 0;
  rrt._goal_state = 20;
  rrt._step_size = 1;
  rrt._max_iter = 10;

  bool success = run_RRT(rrt);

  // struct Path* path = rrt._path;

  printf("Hello word\n");
  return 0;
}
