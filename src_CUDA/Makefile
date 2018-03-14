NVCCFLAGS = -arch=compute_61 -code=sm_61 -Xptxas "-v" -I.
NVCCLDFLAGS = -L /usr/local/cuda/lib64
NVCCLIBS = -lcudart

CFLAGS=-I /usr/local/cuda/include -I. -Wall -g -lm
DEPS = PathPlanning.h RRT.h
OBJ = PathPlanning.o RRT.o RRTtest.o

#%.o: %.c $(DEPS)
#	gcc -c -o $@ $< $(CFLAGS)

#%.o: %.c $(DEPS)
#	nvcc $(NVCCFLAGS) $(NVCCLDFLAGS) -o $@ $< $(NVCCLIBS)

#RRTtest: $(OBJ)
#	gcc -o $@ $^ $(CFLAGS)

RRTtest: $(OBJ)
	nvcc $(NVCCFLAGS) $(NVCCLDFLAGS) -o $@ $^ $(NVCCLIBS)

#RRTtest.o: RRTtest.c
#	gcc -c -o $@ $^ $(CFLAGS)

#PathPlanning.o: PathPlanning.c
#	gcc -c -o $@ $^ $(CFLAGS)

#RRT.o: RRT.c
#	gcc -c -o $@ $^ $(CFLAGS)

RRTtest.o: RRTtest.cu
	nvcc $(NVCCFLAGS) -c -o $@ $^

PathPlanning.o: PathPlanning.cu
	nvcc $(NVCCFLAGS) -c -o $@ $^

RRT.o: RRT.cu
	nvcc $(NVCCFLAGS) -c -o $@ $^

clean:
	rm -rf *.o RRTtest
