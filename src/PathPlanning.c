#include <stdlib.h>
#include <math.h>
#include "PathPlanning.h"

/* Number of sample points for valid checker. */
#define NUMSAMPLE 1000
/* The distance square threshold of being close enough*/
#define DIST2_THRESHOLD 0.01

static bool checkBoundary(pp*, float, float, float);
static bool checkValid(pp*, ppstate*);

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

static bool
checkValid(pp *pp, ppstate *state)
{
  return pp->_validFunc(state);
}

/* Check if a transition from start state to end state if valid. */
bool
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

  /* TODO: think of ways to parallelize here. */
  isTransitionValid_loop: for (uint32_t num = 0; num < NUMSAMPLE; num++)
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
  ppstate* out = malloc(sizeof(ppstate));
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
  ppstate *out = malloc(sizeof(ppstate));
  assert (out != NULL);
  copyState(out, &pp->_goal_state);
  return out;
}
