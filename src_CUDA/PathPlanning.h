#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <cuda_runtime.h>

#define CUDA_ERR_CK(expr) \
  {                            \
    cudaError_t err = expr;    \
    if (err != cudaSuccess)    \
    {                          \
      printf("CUDA call failed in file %s at line %d!\n%s\n", \
             __FILE__, __LINE__, cudaGetErrorString(err)); \
      exit(1);                 \
    }                          \
  }

#define USE_CUDA true
// #define NUMSAMPLE 1048576
#define NUMSAMPLE 512
//number of points and triangles in the meshes
//#define ROBOT_NUM_TRIANGLES 44
//#define ROBOT_NUM_POINTS 132
#define ROBOT_NUM_POINTS 132
//#define OBSTACLE_NUM_TRIANGLES 
//#define OBSTACLE_NUM_POINTS 3744
#define OBSTACLE_NUM_POINTS 1872

//some vector macros for triangle test
/*#define CROSS(dest,v1,v2){                     \
              dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
              dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
              dest[2]=v1[0]*v2[1]-v1[1]*v2[0];}*/

#define CROSS(dest,v1,v2){                     \
              dest.x=v1.y*v2.z-v1.z*v2.y; \
              dest.y=v1.z*v2.x-v1.x*v2.z; \
              dest.z=v1.x*v2.y-v1.y*v2.x;}

#define DOT(v1,v2) (v1.x*v2.x+v1.y*v2.y+v1.z*v2.z)

#define SUB(dest,v1,v2){         \
            dest.x=v1.x-v2.x; \
            dest.y=v1.y-v2.y; \
            dest.z=v1.z-v2.z;}

#define ADD(dest,v1,v2){         \
            dest.x=v1.x+v2.x; \
            dest.y=v1.y+v2.y; \
            dest.z=v1.z+v2.z;}

#define FABS(x) (float)(fabs(x)) 

extern float *robot_p;
extern float *obstacle_p;

extern float *dev_robot_p;
extern float *dev_obstacle_p;

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
  FILE *_out_file;
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
void setOutFile(pp*, FILE*);

/* Helper functions. */
bool isTransitionValid(pp*, ppstate*, ppstate*);
bool isGoalSatisfied(pp*, ppstate*);
float randFloat(uint8_t);
float calDistSqr(ppstate*, ppstate*);
float calDist(ppstate*, ppstate*);
void copyState(ppstate*, ppstate*);
ppstate* sampleUniform(pp*);
ppstate* sampleGoal(pp*);

__device__ bool tempValid(ppstate*);
__device__ bool tempValid_cuda(ppstate*, float*, float*);

#endif /* PathPlanning.h */
