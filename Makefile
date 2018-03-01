CC=gcc
CFLAGS=-I

RRT_test: RRT_test.o
	$(CC) -o RRT_test RRT_test.c
