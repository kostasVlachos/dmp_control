#!/bin/bash

# ==================================
# define some colors for output
# ==================================
COLOR_RED="\033[1;31m"
COLOR_GREEN="\033[1;32m"
COLOR_YELLOW="\033[1;33m"
COLOR_BLUE="\033[1;34m"
COLOR_CYAN="\033[1;36m"
COLOR_WHITE="\033[1;37m"
COLOR_RESET="\033[0m"

cd $INSTALL_SCRIPTS_DIR
AS64_ERROR=0

if [ $# -eq 0 ]; then
  declare -a arr=("frilibrary") # "eigen3" "armadillo")
elif [ "$1" = "-a" ]; then
  declare -a arr=("latest_armadillo" "latest_eigen3" "frilibrary" "ati_sensor" "barretthand" "mujoco" "ros_deps")
else
  echo -e $COLOR_YELLOW"The provided option does not exist..."$COLOR_RESET
  exit 1
fi

echo -e $COLOR_BLUE"Installing as64_ws..."$COLOR_RESET

#echo -e $COLOR_BLUE"Installing main Dependencies: cmake, wget, xz-utils..."$COLOR_RESET
#sudo apt-get update > /dev/null
#sudo apt-get install -y build-essential cmake wget xz-utils unzip > /dev/null

mkdir -p deps

## now loop through the above array
for i in "${arr[@]}"
do
  cd $INSTALL_SCRIPTS_DIR
  source install_$i.sh
  if [ $AS64_ERROR -ne 0 ]; then
    echo -e $COLOR_RED"Failed to install as64_ws Packages...."$COLOR_RESET
    exit 1
  fi
  sleep 4
done

cd $INSTALL_SCRIPTS_DIR
rm -rf deps/
cd ..

echo -e $COLOR_GREEN"as64_ws Packages Successfully installed!"$COLOR_RESET
