#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "RRT.h"
#include "PathPlanning.h"

bool stateValidFunc (ppstate*);

int main()
{
  struct pathPlanning rrt;
  setBoundary(&rrt, 5.0);
  setValidFunc(&rrt, stateValidFunc);
  setStartState(&rrt, 0.0, 0.0, 0.0);
  setGoalState(&rrt, 5.0, 5.0, 5.0);
  setStepSize(&rrt, 0.1);
  setMaxIter(&rrt, 100);
  setBias(&rrt, 0.1);

  srand(time(NULL));
  bool success = run_RRT(&rrt);
  bool valid = isTransitionValid(&rrt, &rrt._start_state, &rrt._goal_state);

  // struct Path* path = rrt._path;

  printf("Hello world\ngoal_state_y: %.3f\nsuccess: %d\nvalid: %d\n",
         rrt._goal_state.y, success, valid);

  printf("rand1:%.4f\nrand2:%.4f\nrand3:%.4f\n", randFloat(3), randFloat(3),
         randFloat(3));

  printf("Satisfied:%d\n", isGoalSatisfied(&rrt, &rrt._start_state));
  return 0;
}

/* A valid motion checker. */
bool
stateValidFunc (ppstate* state)
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
