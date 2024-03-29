#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"
echo $SCRIPT_PATH
if [ -x "$(whereis nvim | awk '{print $2}')" ]; then
 VIM_BIN="$(whereis nvim | awk '{print $2}')"
 HEADLESS="--headless"
elif [ -x "$(whereis vim | awk '{print $2}')" ]; then
 VIM_BIN="$(whereis vim | awk '{print $2}')"
 HEADLESS=""
fi

$VIM_BIN -u "$GIT_PATH/linux-setup/submodules/profile_manager/epigen/epigen.vimrc" $HEADLESS -Ens -c "%s/uav[0-9]\+/$1/g" -c "wqa" -- "$SCRIPT_PATH/../rviz/calibration.rviz"
$VIM_BIN -u "$GIT_PATH/linux-setup/submodules/profile_manager/epigen/epigen.vimrc" $HEADLESS -Ens -c "%s/uav[0-9]\+/$1/g" -c "wqa" -- "$SCRIPT_PATH/../rviz/config.rviz"
