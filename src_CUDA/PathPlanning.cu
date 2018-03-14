#include <stdlib.h>
#include <math.h>
#include "PathPlanning.h"

/* Number of sample points for valid checker. */
#define NUMSAMPLE 1024
/* The distance square threshold of being close enough*/
#define DIST2_THRESHOLD 0.01

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

/* Kernel to boost the valid checking process. */
__global__ void
validCheckKernel(pp *pp, float x1, float y1, float z1,
                 float stepx, float stepy, float stepz, bool *result)
{
  int i = blockDim.x * blockIdx.x + threadIdx.x;
  __shared__ bool temp;
  temp = true;
  __syncthreads();

  ppstate checkState;
  checkState.x = x1 + i * stepx;
  checkState.y = y1 + i * stepy;
  checkState.z = z1 + i * stepz;
  if (!tempValid(&checkState))
    temp = false;

  __syncthreads();
  *result = temp;

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
                                                       result);

  CUDA_ERR_CK(cudaMemcpy(&local_res, result, sizeof(bool),
                         cudaMemcpyDeviceToHost));
  CUDA_ERR_CK(cudaFree(result));

  return local_res;
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
