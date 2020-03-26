#!/bin/bash
#=============================================================================
# common functions in the gemmini utilities
#=============================================================================
__print() {
	local color="$1"
	local msg="$2"
	case $color in
		red)    echo -e "\e[31m${msg}\e[0m" ;;
		green)  echo -e "\e[32m${msg}\e[0m" ;;
		yellow) echo -e "\e[33m${msg}\e[0m" ;;
		cyan)   echo -e "\e[36m${msg}\e[0m" ;;
		*) echo "invalid color $color!"; exit 1;;
	esac
}
err() {  __print "red"    "[ERROR]: $@"; exit 1; }
warn() { __print "yellow" "[WARN]: $@"; }
info() { __print "cyan"   "[INFO]: $@"; }
ok() {   __print "green"  "[OK]: $@"; }

ferr() {  echo "[ERROR]: $1" >> "$2";  err  "$1"; }
fwarn() { echo "[WARN]: $1"  >> "$2";  warn "$1"; }
finfo() { echo "[INFO]: $1"  >> "$2";  info "$1"; }
fok() {   echo "[OK]: $1"    >> "$2";  ok   "$1"; }

