#!/usr/bin/env bash

LOGFILE=test_output.txt

MNK_VALUES="
16  16  16
32  32  32
64  64  64
128 128 128
...
10000 10000 10000
"

echo "=============TEST RESULTS=============" > $LOGFILE

# this version does A*B+D=C
/root/ee290/gemm-linux -verify $MNK_VALUES | tee -a $LOGFILE

# this version does A*B=C
/root/ee290/gemm-linux -verify -no_d $MNK_VALUES | tee -a $LOGFILE

cat $LOGFILE
poweroff -f
