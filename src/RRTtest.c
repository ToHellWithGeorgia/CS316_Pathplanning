#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "RRT.h"
#include "PathPlanning.h"

bool cubeCenterValidFunc (ppstate*);

int main()
{
  struct pathPlanning rrt;
  setBoundary(&rrt, 5.0);
  setValidFunc(&rrt, cubeCenterValidFunc);
  setStartState(&rrt, 0.0, 0.0, 0.0);
  setGoalState(&rrt, 5.0, 5.0, 5.0);
  setStepSize(&rrt, 0.1);
  setMaxIter(&rrt, 1000);
  setBias(&rrt, 0.1);

  FILE *fd;
  fd = fopen("solution.dat", "w");
  setOutFile(&rrt, fd);

  srand(time(NULL));
  run_RRT(&rrt);

  /*
  bool valid = isTransitionValid(&rrt, &rrt._start_state, &rrt._goal_state);
  printf("Hello world\ngoal_state_y: %.3f\nsuccess: %d\nvalid: %d\n",
         rrt._goal_state.y, success, valid);

  printf("rand1:%.4f\nrand2:%.4f\nrand3:%.4f\n", randFloat(3), randFloat(3),
         randFloat(3));

  printf("Satisfied:%d\n", isGoalSatisfied(&rrt, &rrt._start_state));

  interpolate(&rrt._start_state, &rrt._goal_state, rrt._step_size);
  printf("Normed x:%f, y:%f, z:%f", rrt._goal_state.x, rrt._goal_state.y,
         rrt._goal_state.z);
  */

  fclose(fd);
  return 0;
}

/* A valid motion checker to model a cube in the center of the graph. */
bool
cubeCenterValidFunc (ppstate* state)
{
  float x = state->x;
  float y = state->y;
  float z = state->z;

  /* Define a obstacle square in the middle of the cube. */
  if ((x > 2) && (x < 3) &&
      (y > 2) && (y < 3) &&
      (z > 2) && (z < 3))
    return false;
  else
    return true;
}
