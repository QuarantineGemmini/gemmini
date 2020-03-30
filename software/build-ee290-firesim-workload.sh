#!/bin/bash

DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
FSIMSW="$DIR/../../../sims/firesim/sw/firesim-software"

# This script will rebuild the gemmini-rocc-tests, and then
# build the appropriate Linux Image for FireSim execution using
# FireMarshal
echo "Building gemmini-rocc-tests EE290 FireSim Workload"

if [ "$#" -ne 1 ]; then
  echo "[ERROR]: usage: $0 <orig_tiler|fsm_tiler|hw_tiler>"
  exit 1
fi
WORKLOAD=$1

cd "$DIR/gemmini-rocc-tests"
buildworkload_gemmini -force -noisy $WORKLOAD
cp -r build_${WORKLOAD}/* ../overlay/root/

cd $FSIMSW
rm -f images/gemmini-tests-ee290*
./marshal -v --workdir $DIR build gemmini-tests-ee290.json
./marshal -v --workdir $DIR install gemmini-tests-ee290.json
