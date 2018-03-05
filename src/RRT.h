#ifndef RRT_H
#define RRT_H

#include <stdbool.h>
#include <stdint.h>
#include <PathPlanning.h>

/* The size of the struct is now 8B */
struct rrt_node
{
  ppstate _pos;       /* Store the positions of current node. */
  uint32_t _parent;   /* Index of the parent node. */
};

typedef struct rrt_node node;

bool run_RRT (struct pathPlanning*);

#endif /* RRT.h */
