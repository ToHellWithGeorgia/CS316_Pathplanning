#include <stdlib.h>
#include <assert.h>
#include "RRT.h"
#include "PathPlanning.h"

#define INIT_SIZE 128

static node *tree;
static uint32_t node_cnt;   /* Count the numbers of nodes in trees. */
static uint32_t tree_size;  /* Keep tracks of how big is the tree. */

static void RRT_init(pp*);
static void expand_tree_size(void);
static uint32_t findNN(ppstate*);
static void interpolate(ppstate*, ppstate*, float);

/* Init a 1k tree first. We will grow the array if it needs more size. */
static void
RRT_init(pp* pp)
{
  tree = (node *)calloc(INIT_SIZE, sizeof(node));
  assert (tree);
  tree_size = INIT_SIZE;
  node_cnt = 0;

  /* Push the start node to the array. It has itself as its parent. */
  copyState(&tree[0]._pos, &pp->_start_state);
  tree[0]._parent = 0;
  node_cnt += 1;
}

/* Expand the tree by twice the original size. */
static void
expand_tree_size()
{
  tree_size *= 2;
  tree = (node *)realloc(tree, tree_size * sizeof(node));
  assert (tree);
}

/* Find the nearest neighbour in the tree now to the target.
   TODO: this part should be parellelizable. */
static uint32_t
findNN(ppstate *tgt)
{
  return 0;
}

/* Interpolate the target points to be within the distance of the 
   original points.*/
static void
interpolate(ppstate *org, ppstate *tgt, float max_dist)
{
  float dist = calDist(org, tgt);
  if (dist <= max_dist)
    return;

  float xvec = tgt->x - org->x;
  float yvec = tgt->y - org->y;
  float zvec = tgt->z - org->z;

  tgt->x += xvec / dist * max_dist;
  tgt->y += yvec / dist * max_dist;
  tgt->z += zvec / dist * max_dist;
}

/* Grow the tree until the max_iter */
bool
run_RRT (struct pathPlanning *pp)
{
  RRT_init(pp);
 
  uint32_t iter_cnt = 0;
  bool reached = false;

  /* TODO: This while loop could be parellelized. Maybe create multiple
     initial trees? */
  while (iter_cnt < pp->_max_iter)
  {
    ppstate *new_node;
    /* Grow the tree with a bias to the goal. */
    if (randFloat(1) > pp->_bias)
      new_node = sampleUniform(pp);
    else
      new_node = sampleGoal(pp);

    /* Find the nearest neighbour in the existing nodes. */
    uint32_t nn_idx = findNN(new_node);
  }

  return true;
}
