#!/bin/bash
rm -rf build/ install/ log/
colcon build --symlink-install --continue-on-error
source install/setup.bash