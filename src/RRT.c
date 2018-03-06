#include <stdlib.h>
#include <stdio.h>
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
static void write_state(FILE*, ppstate*);

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
  printf("Expanding the tree from size %d to %d.\n", tree_size, tree_size * 2);
  tree_size *= 2;
  tree = (node *)realloc(tree, tree_size * sizeof(node));
  assert (tree);
}

/* Find the nearest neighbour in the tree now to the target.
   TODO: this part should be parellelizable. */
static uint32_t
findNN(ppstate *tgt)
{
  /* We will use a brutal force search for now. Maybe use the kd-tree */
  assert (node_cnt > 0);
  uint32_t find_idx = UINT32_MAX;
  float distSqr = SIZE_MAX;

  for (uint32_t idx = 0; idx < node_cnt; idx++)
  {
    float newDistSqr = calDistSqr(&tree[idx]._pos, tgt);
    if (newDistSqr < distSqr)
    {
      find_idx = idx;
      distSqr = newDistSqr;
    }
  }
  return find_idx;
}

/* Interpolate the target points to be within the distance of the 
   original points.*/
void
interpolate(ppstate *org, ppstate *tgt, float max_dist)
{
  float dist = calDist(org, tgt);
  if (dist <= max_dist)
    return;

  float xvec = tgt->x - org->x;
  float yvec = tgt->y - org->y;
  float zvec = tgt->z - org->z;

  tgt->x = org->x + xvec / dist * max_dist;
  tgt->y = org->y + yvec / dist * max_dist;
  tgt->z = org->z + zvec / dist * max_dist;
}

/* Write the ppstate to the file. */
void
write_state(FILE *fd, ppstate *s)
{
  fprintf(fd, "%.6f %.6f %.6f\n", s->x, s->y, s->z);
}

/* Grow the tree until the max_iter */
bool
run_RRT (struct pathPlanning *pp)
{
  RRT_init(pp);
 
  uint32_t iter_cnt = 0;
  // bool reached = false;

  /* TODO: This while loop could be parellelized. Maybe create multiple
     initial trees? */
  while (iter_cnt < pp->_max_iter)
  {
    iter_cnt++;

    ppstate *new_state;
    /* Grow the tree with a bias to the goal. */
    if (randFloat(1) > pp->_bias)
      new_state = sampleUniform(pp);
    else
      new_state = sampleGoal(pp);

    /* Find the nearest neighbour in the existing nodes. */
    uint32_t nn_idx = findNN(new_state);
    assert (nn_idx < node_cnt);
    /* Interpolate the new state to be within the reach. */
    interpolate(&tree[nn_idx]._pos, new_state, pp->_step_size);

    /* If the transition is valid, push to the tree. */
    if (isTransitionValid(pp, &tree[nn_idx]._pos, new_state))
    {
      copyState(&tree[node_cnt]._pos, new_state);
      tree[node_cnt]._parent = nn_idx;
      node_cnt++;
    } else
    {
      free(new_state);
      continue;
    }

    /* Check if the new node is close enough to the goal. */
    if (isGoalSatisfied(pp, new_state))
    {
      /* Found the solution! Stop the loop and TODO: collect the path. */
      printf("Solution found! Takes %d loops. Need %d steps.\n", iter_cnt,
             node_cnt);
      FILE *fd = pp->_out_file;

      /* Write the goal to the file, then traverse the tree */
      write_state(fd, &pp->_goal_state);
      uint32_t node_idx = node_cnt - 1;
      while (tree[node_idx]._parent != node_idx)
      {
        write_state(fd, &tree[node_idx]._pos);
        node_idx = tree[node_idx]._parent;
      }
      /* Make sure we reach the start point. */
      assert (node_idx == 0);
      write_state(fd, &tree[0]._pos);
      return true;
    }
    free(new_state);

    /* Check if we need to expand the tree. */
    if (node_cnt == tree_size)
      expand_tree_size();
  }

  printf("Solution not found :(\n");
  return false;
}
