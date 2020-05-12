#!/usr/bin/env bash

LOGFILE=test_output.txt

echo "=============TEST RESULTS=============" > $LOGFILE

sizes="32 64 128 192 256 320 384 448 512 576 640 704 768 832 896"
#sizes="32 64 128 192 256 320 384 448 512 576 640 704 768 832 896 960 1024"
#sizes="1024 1152 1280 1408 1536 1664 1792 1920 2048 2304 2560 2816 3072 3328 3584 3840 4096"

for size in $sizes; do
  # this version does A*B+D=C
  /root/ee290/gemm-linux -zeros $size $size $size | tee -a $LOGFILE
done

cat $LOGFILE
poweroff -f
