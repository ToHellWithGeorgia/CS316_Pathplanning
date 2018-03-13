#########################################################
# Provide Accelerator name, source files, and workloads #
#########################################################

ACCEL_NAME = RRTtest
SRCS = $(wildcard *.c)
WORKLOADS = runRRT

# non-gem5 and non-aladdin program-centric flags
CFLAGS += 
LFLAGS += -lm 
INCLUDES += 

#########################################################
# DO NOT MODIFY BELOW UNLESS YOU KNOW WHAT YOU'RE DOING #
#########################################################

export WORKLOAD = $(WORKLOADS)
EXTRA_FLAGS = -DDMA_INTERFACE_V3

# accelerator connection files
GEM5_SRCS = aladdin_sys_connection.cpp aladdin_sys_constants.cpp
# DMA support
GEM5_SRCS += dma_interface.c
# full paths
GEM5_FULL_SRCS = $(GEM5_SRCS:%=$(ALADDIN_HOME)/gem5/%)


GEM5_CFLAGS = -static -O3 \
              -msse -mfpmath=sse \
              -fno-exceptions \
              -std=gnu99

INCLUDES += -I$(ALADDIN_HOME)/gem5

# by default, build the cpu version
all: gem5-cpu

.PHONY: gem5-cpu gem5-accel gem5-accel-dma clean-gem5 clean

# Builds the standalone CPU version only (no DMA)
gem5-cpu: $(SRCS) $(GEM5_FULL_SRCS)
	@echo "[building $(ACCEL_NAME) for gem5 (NO ACCELERATION)]"
	@$(CC) $(GEM5_CFLAGS) $(CFLAGS) $(GEM5_FULL_SRCS) $(SRCS) $(INCLUDES) $(EXTRA_FLAGS) -o $(ACCEL_NAME) $(LFLAGS)
	@echo "[trace binary]"
	$(MAKE) -C . trace-binary
	@echo "[run trace]"
	$(MAKE) -C . run-trace

# Builds the gem5 version with HW acceleration turned on (no DMA)
gem5-accel: $(SRCS) $(GEM5_FULL_SRCS)
	@echo "[trace binary]"
	$(MAKE) -C . trace-binary
	@echo "[run trace]"
	$(MAKE) -C . run-trace
	@echo "[building $(ACCEL_NAME) for gem5 with HW acceleration]"
	@$(CC) $(GEM5_CFLAGS) $(CFLAGS) -DGEM5_ACCEL $(SRCS) $(GEM5_FULL_SRCS) $(INCLUDES) $(EXTRA_FLAGS) -o $(ACCEL_NAME) $(LFLAGS)	

# Builds the gem5 version with HW acceleration turned on (with DMA)
gem5-accel-dma: $(SRCS) $(GEM5_FULL_SRCS)
	@echo "[trace binary]"
	$(MAKE) -C . DMA_MODE=1 trace-binary
	@echo "[run trace]"
	$(MAKE) -C . run-trace
	@echo "[building $(ACCEL_NAME) for gem5 with HW acceleration]"
	@$(CC) $(GEM5_CFLAGS) $(CFLAGS) -DGEM5_ACCEL -DDMA_MODE $(SRCS) $(GEM5_FULL_SRCS) $(INCLUDES) $(EXTRA_FLAGS) -o $(ACCEL_NAME) $(LFLAGS)	

clean-gem5:
	rm -f $(ACCEL_NAME)

clean: clean-gem5 clean-trace
	rm -rf outputs
	rm -f stdout.gz
	rm -f labelmap
	rm -f out.csv
	rm -f $(ACCEL_NAME)_summary
	rm -f $(ACCEL_NAME)_power_stats.txt
	rm -f dddg_*


include ../../common/Makefile.tracer
