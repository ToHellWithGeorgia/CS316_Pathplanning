#!/usr/bin/env bash

rrt_home=${ALADDIN_HOME}/gem5-aladdin-examples/CS316_Pathplanning/src
gem5_dir=${ALADDIN_HOME}/../..

${gem5_dir}/build/X86/gem5.opt \
  --debug-flags=HybridDatapath,Aladdin \
  --outdir=${rrt_home}/outputs \
  ${gem5_dir}/configs/aladdin/aladdin_se.py \
  --num-cpus=1 \
  --enable_prefetchers \
  --mem-size=4GB \
  --mem-type=DDR3_1600_8x8  \
  --sys-clock=1GHz \
  --cpu-type=DerivO3CPU \
  --caches \
  --cacheline_size=64 \
  --accel_cfg_file=${mac_home}/mac_aladdin_config.cfg \
  -c ${mac_home}/RRTtest \
  | gzip -c > stdout.gz
