#!/usr/bin/env bash

LOGFILE=test_output.txt

/root/ee290/gemm-linux -verify \
  16  16  16 \
  32  32  32 \
  64  64  64 \
  128 128 128 \
  ... \
  10000 10000 10000 \
  | tee -a $LOGFILE

cat $LOGFILE
poweroff -f
