#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <stdbool.h>
#include <stdint.h>

/* path planning state */
typedef ppstate uint32_t

struct pathPlanning
{
  ppstate _start_state;
  ppstate _goal_state;
  uint32_t _step_size;
  uint32_t _max_iter;

  // struct Path *_path;
}

#endif /* PathPlanning.h */
