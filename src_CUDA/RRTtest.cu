#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "RRT.h"
#include "PathPlanning.h"

/* Header to support CUDA. */
#include <cuda_runtime.h>


bool cubeCenterValidFunc (ppstate*);
bool fourCubeValidFunc (ppstate*);
bool simpleMazeValidFunc (ppstate*);

bool triangleMeshValidFunc(ppstate*);

int main(int argc, char **argv)
{
  /* Setting up the triangle obstacles. */
  size_t size_robot_p = 3 * ROBOT_NUM_POINTS * sizeof(float);
  size_t size_obstacle_p = 3 * ROBOT_NUM_POINTS * sizeof(float);
  robot_p = (float *) calloc(3 * ROBOT_NUM_POINTS, sizeof(float));
  obstacle_p = (float *) calloc(3 * OBSTACLE_NUM_POINTS, sizeof(float));

  CUDA_ERR_CK(cudaMalloc((void **)&dev_robot_p, size_robot_p));
  CUDA_ERR_CK(cudaMalloc((void **)&dev_obstacle_p, size_obstacle_p));

  FILE *file;
  //Read in robot triangle mesh points from file
  file = fopen("robot_p.txt", "r");
  int i=0;
  for(i=0; i<3*ROBOT_NUM_POINTS; i++)
    fscanf(file,"%f", &robot_p[i]);
  fclose(file);

  //Read in obstacle triangle mesh points
  file = fopen("obstacle_p.txt", "r");
  for(i=0; i<3*OBSTACLE_NUM_POINTS; i++)
    fscanf(file,"%f", &obstacle_p[i]);
  fclose(file);

  CUDA_ERR_CK(cudaMemcpy(dev_robot_p, robot_p, size_robot_p,
                         cudaMemcpyHostToDevice));
  CUDA_ERR_CK(cudaMemcpy(dev_obstacle_p, obstacle_p, size_obstacle_p,
                         cudaMemcpyHostToDevice));
  /* Finish setting up triangle obstacles. */

  struct pathPlanning rrt;
  setBoundary(&rrt, 5.0);
  // setValidFunc(&rrt, cubeCenterValidFunc);
  // setValidFunc(&rrt, eightCubeValidFunc);
  setValidFunc(&rrt, triangleMeshValidFunc);
  setStartState(&rrt, 0.0, 0.0, 0.0);
  setGoalState(&rrt, 5.0, 5.0, 5.0);
  setStepSize(&rrt, 0.1);
  setMaxIter(&rrt, 8000);
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

/* A slightly more complicated valid motion checker. */
bool
eightCubeValidFunc (ppstate* state)
{
  float x = state->x;
  float y = state->y;
  float z = state->z;

  if ((x > 1) && (x < 2) &&
      (y > 1) && (y < 2) &&
      (z > 1) && (z < 2))
    return false;

  if ((x > 1) && (x < 2) &&
      (y > 1) && (y < 2) &&
      (z > 3) && (z < 4))
    return false;

  if ((x > 1) && (x < 2) &&
      (y > 3) && (y < 4) &&
      (z > 1) && (z < 2))
    return false;

  if ((x > 1) && (x < 2) &&
      (y > 3) && (y < 4) &&
      (z > 3) && (z < 4))
    return false;

  if ((x > 3) && (x < 4) &&
      (y > 1) && (y < 2) &&
      (z > 1) && (z < 2))
    return false;

  if ((x > 3) && (x < 4) &&
      (y > 1) && (y < 2) &&
      (z > 3) && (z < 4))
    return false;

  if ((x > 3) && (x < 4) &&
      (y > 3) && (y < 4) &&
      (z > 1) && (z < 2))
    return false;

  if ((x > 3) && (x < 4) &&
      (y > 3) && (y < 4) &&
      (z > 3) && (z < 4))
    return false;

  return true;
}

/* A simple maze. */
bool
simpleMazeValidFunc(ppstate *state)
{
  float x = state->x;
  float z = state->z;

  if ((x > 1) && (x < 2) &&
      (z >= 0) && (z < 4))
    return false;

  if ((x > 3) && (x < 4) &&
      (z > 1) && (z <= 5))
    return false;

  return true;
}

bool
triangleMeshValidFunc(ppstate *state)
{
  float x_pos = state->x;
  float y_pos = state->y;
  float z_pos = state->z;
  ppstate A;
  ppstate B;
  ppstate C;
  ppstate D;
  ppstate E;
  ppstate F;
  ppstate AB_cross_AC;
  ppstate A_B;
  ppstate A_C;
  ppstate P;
  ppstate Q;
  ppstate Q_P;
  ppstate A_P;
  ppstate QP_cross_AP;
  int robot_tri = 0;
  for (robot_tri = 0; robot_tri<3 * ROBOT_NUM_POINTS; robot_tri += 9) {
    /*float A[3];
    float B[3];
    float C[3];
    float A_B[3];
    float A_C[3];
    float AB_cross_AC[3];*/
    // coords of pt A of robot triangle
    A.x = robot_p[robot_tri] + x_pos;
    A.y = robot_p[robot_tri + 1] + y_pos;
    A.z = robot_p[robot_tri + 2] + z_pos;
    // coords of pt B of robot triangle
    B.x = robot_p[robot_tri + 3] + x_pos;
    B.y = robot_p[robot_tri + 4] + y_pos;
    B.z = robot_p[robot_tri + 5] + z_pos;
    // coords of pt C of robot triangle
    C.x = robot_p[robot_tri + 6] + x_pos;
    C.y = robot_p[robot_tri + 7] + y_pos;
    C.z = robot_p[robot_tri + 8] + z_pos;

    SUB(A_B, B, A);
    SUB(A_C, C, A);
    CROSS(AB_cross_AC, A_B, A_C);

    //Now we iterate over obstacle triangles
    int obstacle_tri = 0;
    for (obstacle_tri = 0; obstacle_tri<3 * OBSTACLE_NUM_POINTS; obstacle_tri += 9) {

      /*float D[3];
      float E[3];
      float F[3];*/

      // coords of pt A of obstacle triangle
      D.x = obstacle_p[obstacle_tri];
      D.y = obstacle_p[obstacle_tri + 1];
      D.z = obstacle_p[obstacle_tri + 2];
      // coords of pt B of obstacle triangle
      E.x = obstacle_p[obstacle_tri + 3];
      E.y = obstacle_p[obstacle_tri + 4];
      E.z = obstacle_p[obstacle_tri + 5];
      // coords of pt C of obstacle triangle
      F.x = obstacle_p[obstacle_tri + 6];
      F.y = obstacle_p[obstacle_tri + 7];
      F.z = obstacle_p[obstacle_tri + 8];

      //iterate through edges of obstacle triangle and perform line-triangle intersection tests
      int edge = 0;
      /*float P[3];
      float Q[3];
      float A_P[3];
      float Q_P[3];
      float QP_cross_AP[3];*/
      float t_num;
      float t_denom;
      float x_num;
      float y_num;
      for (edge = 0;edge<3;edge++) {
        if (edge == 0) {
          //P = D;
          //Q = E;
          /*int dim;
          for (dim = 0;dim<3;dim++) {
            P[dim] = D[dim];
            Q[dim] = E[dim];
          }*/
          P = D;
          Q = E;
          SUB(A_P, P, A);
          t_num = DOT(A_P, AB_cross_AC);
        }
        else if (edge == 1) {
          //Q = F;
          /*int dim;
          for (dim = 0;dim<3;dim++)
            Q[dim] = F[dim];*/
          Q=F;

        }
        else {
          //P = E;
          /*int dim;
          for (dim = 0;dim<3;dim++)
            P[dim] = E[dim];*/
          P = E;
          SUB(A_P, P, A);
          t_num = DOT(A_P, AB_cross_AC);
        }
        SUB(Q_P, P, Q);
        t_denom = DOT(Q_P, AB_cross_AC);
        // if 0 <= t <= 1
        if (((t_num == 0.0f) & (FABS(t_denom) > 0.0f)) | ((FABS(t_num) <= FABS(t_denom)) & ~((t_num < 0.0f) ^ (t_denom < 0.0f)) & (FABS(t_denom) > 0.0f))) {
          CROSS(QP_cross_AP, Q_P, A_P);
          x_num = DOT(A_C, QP_cross_AP);
          y_num = -DOT(A_B, QP_cross_AP);
          // 0 <= x <= 1?
          bool cond1 = ((x_num == 0.0f) | ((FABS(x_num) <= FABS(t_denom)) & ~((x_num < 0.0f) ^ (t_denom < 0.0f))));
          // 0 <= y <= 1?
          bool cond2 = ((y_num == 0.0f) | ((FABS(y_num) <= FABS(t_denom)) & ~((y_num < 0.0f) ^ (t_denom < 0.0f))));
          // (x+y) <= 1?
          float sum = x_num + y_num;
          bool cond3 = ((FABS(sum) <= FABS(t_denom)) | (t_denom < 0.0f));
          if (cond1 & cond2 & cond3) {
            return false;
          }
        }
      }

    }
  }
  
  return true;
}
