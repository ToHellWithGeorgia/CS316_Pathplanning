CC=gcc
CFLAGS=-I. -Wall -g -lm
DEPS = PathPlanning.h RRT.h
OBJ = PathPlanning.o RRT.o RRTtest.o
OBJDIR = obj

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

RRTtest: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm -rf *.o RRTtest
