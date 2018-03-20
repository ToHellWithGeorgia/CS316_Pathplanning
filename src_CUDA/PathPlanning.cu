#include <stdlib.h>
#include <math.h>
#include "PathPlanning.h"

/* Number of sample points for valid checker. */
// #define NUMSAMPLE 1048576
#define NUMSAMPLE 1024
/* The distance square threshold of being close enough*/
#define DIST2_THRESHOLD 0.01

float *robot_p;
float *obstacle_p;

float *dev_robot_p;
float *dev_obstacle_p;

static bool checkBoundary(pp*, float, float, float);
// bool checkValid(pp*, ppstate*);

static bool
checkBoundary(pp* pp, float x, float y, float z)
{
  float limit = pp->_side_max; 
  if ((x >= 0) && (y >= 0) && (z >= 0) &&
      (x <= limit) && (y <= limit) && (z <= limit))
    return true;
  else
    return false;
}

/*------------------ Setup functions. --------------------*/

/* Define the boundary as a cube of side length limit from
   the original point. */
void
setBoundary(pp *pp, float limit)
{
  pp->_side_max = limit;
}

/* Set the valid state checker. */
void
setValidFunc(pp *pp, bool (*validFunc)(ppstate*))
{
  pp->_validFunc = validFunc;
}

void
setStartState(pp *pp, float x, float y, float z)
{
  assert (checkBoundary(pp, x, y, z));
  pp->_start_state.x = x;
  pp->_start_state.y = y;
  pp->_start_state.z = z;
  assert (pp->_validFunc(&pp->_start_state));
}

void 
setGoalState(pp *pp, float x, float y, float z)
{
  assert (checkBoundary(pp, x, y, z));
  pp->_goal_state.x = x;
  pp->_goal_state.y = y;
  pp->_goal_state.z = z;
  assert (pp->_validFunc(&pp->_start_state));
}

void
setStepSize(pp *pp, float step)
{
  pp->_step_size = step;
}

void
setMaxIter(pp *pp, uint32_t iter)
{
  pp->_max_iter = iter;
}

void
setBias(pp* pp, float bias)
{
  pp->_bias = bias;
}

void
setOutFile(pp* pp, FILE *fd)
{
  pp->_out_file = fd;
}

/* ------------------- Helper Functions ------------------*/

__host__ __device__ bool
checkValid(pp *pp, ppstate *state)
{
  return pp->_validFunc(state);
}

/* Cannot use the host valid checker code. Workaround. */
//__device__ bool
//tempValid(ppstate* state);

/* Kernel to boost the valid checking process. */
__global__ void
validCheckKernel(pp *pp, float x1, float y1, float z1,
                 float stepx, float stepy, float stepz, bool *result,
                 float *dev_rob, float *dev_obst)
{
  int i = blockDim.x * blockIdx.x + threadIdx.x;
  // __shared__ bool temp;
  // / temp = true;
  // __syncthreads();

  ppstate checkState;
  checkState.x = x1 + i * stepx;
  checkState.y = y1 + i * stepy;
  checkState.z = z1 + i * stepz;
  /*
  if (!tempValid(&checkState))
    *result = false;
  */
  if (!tempValid_cuda(&checkState, dev_rob, dev_obst))
    *result = false;

  // __syncthreads();
  // *result = temp;
}

/* Check if a transition from start state to end state if valid. */
__host__ bool
isTransitionValid(pp *pp, ppstate *start, ppstate *end)
{
  /* Return false if either of the points is in the obstacle region. */
  if (!checkValid(pp, start) || !checkValid(pp, end))
    return false;

  float x1 = start->x;
  float y1 = start->y;
  float z1 = start->z;
  float x2 = end->x;
  float y2 = end->y;
  float z2 = end->z;
  assert(checkBoundary(pp, x1, y1, z1) &&
         checkBoundary(pp, x2, y2, z2));

  /* We will use a stupid but parallelizable way to check the state.
     We will sample many states from start and end and check if all of
     them are valid.*/
  float stepx = (x2 - x1) / NUMSAMPLE;
  float stepy = (y2 - y1) / NUMSAMPLE;
  float stepz = (z2 - z1) / NUMSAMPLE;

  bool use_CUDA = true;

  if (use_CUDA)
  {
    /* Setting up the variable to run CUDA kernel. */ 
    bool local_res;
    bool temp = true;
    bool *result;
    CUDA_ERR_CK(cudaMalloc((void **)&result, sizeof(bool)));
    CUDA_ERR_CK(cudaMemcpy(result, &temp, sizeof(bool),
                           cudaMemcpyHostToDevice));

    int numThread = 128;
    validCheckKernel<<<NUMSAMPLE/numThread, numThread>>>(pp, x1, y1, z1,
                                                         stepx, stepy, stepz,
                                                         result, dev_robot_p,
                                                         dev_obstacle_p);

    CUDA_ERR_CK(cudaMemcpy(&local_res, result, sizeof(bool),
                           cudaMemcpyDeviceToHost));
    CUDA_ERR_CK(cudaFree(result));

    return local_res;
  } else
  {
    /* TODO: think of ways to parallelize here. */
    for (uint32_t num = 0; num < NUMSAMPLE; num++)
    {
      ppstate checkState;
      checkState.x = x1 + num * stepx;
      checkState.y = y1 + num * stepy;
      checkState.z = z1 + num * stepz;
      if (!checkValid(pp, &checkState))
        return false;
    }
    return true;
  }
}

/* Check if the state is close enough to the goal */
bool
isGoalSatisfied(pp* pp, ppstate* state)
{
  return calDistSqr(&pp->_goal_state, state) <= DIST2_THRESHOLD;
}

/* Generate a random float number between 0 and max. */
float
randFloat(uint8_t max)
{
  return (float)rand() / (float)RAND_MAX * (float)max;
}

/* Calculate the squared distance between two points */
float
calDistSqr(ppstate* a, ppstate* b)
{
  float gapx = a->x - b->x;
  float gapy = a->y - b->y;
  float gapz = a->z - b->z;

  return gapx * gapx + gapy * gapy + gapz * gapz;
}

/* Calculate the distance between two points. */
float
calDist(ppstate *a, ppstate *b)
{
  return sqrt(calDistSqr(a, b));
}

/* Copy the state src to dst. */
void
copyState(ppstate *dst, ppstate *src)
{
  assert (dst != NULL && src != NULL);
  dst->x = src->x;
  dst->y = src->y;
  dst->z = src->z;
}

/* Sample a random *valid* point in the state space. */
ppstate*
sampleUniform(pp* pp)
{
  ppstate* out = (ppstate *)malloc(sizeof(ppstate));
  assert (out != NULL);
  do
  {
    out->x = randFloat(pp->_side_max);
    out->y = randFloat(pp->_side_max);
    out->z = randFloat(pp->_side_max);
  } while (!checkValid(pp, out));

  return out;
}

/* Sample towards the goal (return a copy of the goal) */
ppstate*
sampleGoal(pp* pp)
{
  ppstate *out = (ppstate *)malloc(sizeof(ppstate));
  assert (out != NULL);
  copyState(out, &pp->_goal_state);
  return out;
}

/*
__device__ bool
tempValid(ppstate* state)
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
*/

__device__ bool
tempValid_cuda(ppstate *state, float *dev_rob, float *dev_obst)
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
    A.x = dev_rob[robot_tri] + x_pos;
    A.y = dev_rob[robot_tri + 1] + y_pos;
    A.z = dev_rob[robot_tri + 2] + z_pos;
    // coords of pt B of robot triangle
    B.x = dev_rob[robot_tri + 3] + x_pos;
    B.y = dev_rob[robot_tri + 4] + y_pos;
    B.z = dev_rob[robot_tri + 5] + z_pos;
    // coords of pt C of robot triangle
    C.x = dev_rob[robot_tri + 6] + x_pos;
    C.y = dev_rob[robot_tri + 7] + y_pos;
    C.z = dev_rob[robot_tri + 8] + z_pos;

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
      D.x = dev_obst[obstacle_tri];
      D.y = dev_obst[obstacle_tri + 1];
      D.z = dev_obst[obstacle_tri + 2];
      // coords of pt B of obstacle triangle
      E.x = dev_obst[obstacle_tri + 3];
      E.y = dev_obst[obstacle_tri + 4];
      E.z = dev_obst[obstacle_tri + 5];
      // coords of pt C of obstacle triangle
      F.x = dev_obst[obstacle_tri + 6];
      F.y = dev_obst[obstacle_tri + 7];
      F.z = dev_obst[obstacle_tri + 8];

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