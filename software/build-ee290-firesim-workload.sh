#!/bin/bash

# This script will rebuild the gemmini-rocc-tests, and then
# build the appropriate Linux Image for FireSim execution using
# FireMarshal
echo "Building gemmini-rocc-tests EE290 FireSim Workload"

DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
FSIMSW="$DIR/../../../sims/firesim/sw/firesim-software"
cd "$DIR/gemmini-rocc-tests"
./build.sh
cp -r build/* ../overlay/root/
cd $FSIMSW
rm images/gemmini-tests-ee290*
./marshal -v --workdir $RDIR build gemmini-tests-ee290.json
./marshal -v --workdir $RDIR install gemmini-tests-ee290.json
cd $RDIR
