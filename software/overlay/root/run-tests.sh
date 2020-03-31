#!/usr/bin/env bash

LOGFILE=test_output.txt

runtest() {
  local desc="$1"
  local name="$2"
  local dir="$3"
  echo "===========$1===========" | tee -a $LOGFILE
  /root/${dir}/${name}-linux | tee -a $LOGFILE
}

echo "=================TEST RESULTS=============" | tee $LOGFILE
runtest "Very Large Matmul"         "very_large_matmul" "ee290"
runtest "Cifar DNN"                 "cifar_quant"       "ee290"
runtest "Multi-Level Perceptron 1"  "mlp1"              "mlps"
runtest "Multi-Level Perceptron 2"  "mlp2"              "mlps"
runtest "MobileNet"                 "mobilenet"         "imagenet"
runtest "ResNet50"                  "resnet50"          "imagenet"

cat $LOGFILE
poweroff -f
