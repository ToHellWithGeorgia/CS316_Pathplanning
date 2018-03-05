#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <stdbool.h>
#include <stdint.h>
#include <assert.h>

/* path planning state */
struct state
{
  float x;
  float y;
  float z;
};

typedef struct state ppstate;

struct pathPlanning
{
  ppstate _start_state;
  ppstate _goal_state;
  float _side_max;    /* The plan space cube boundary*/
  float _step_size;
  uint32_t _max_iter;
  float _bias;
  bool (*_validFunc)(ppstate*);
  // struct Path *_path;
};

typedef struct pathPlanning pp;

/* Setup functions */
void setBoundary(pp*, float);
void setValidFunc(pp*, bool (*validFunc)(ppstate*));
void setStartState(pp*, float, float, float);
void setGoalState(pp*, float, float, float);
void setStepSize(pp*, float);
void setMaxIter(pp*, uint32_t);
void setBias(pp*, float);

/* Helper functions. */
bool isTransitionValid(pp*, ppstate*, ppstate*);
bool isGoalSatisfied(pp*, ppstate*);
float randFloat(uint8_t);
void copyState(ppstate*, ppstate*);
ppstate* sampleUniform(pp*);
ppstate* sampleGoal(pp*);

#endif /* PathPlanning.h */
