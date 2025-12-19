#!/bin/bash
set -e

# ======================================================================
#  update.sh ? Pull latest code, auto-merge, rebuild
# ======================================================================

WS=~/rover_ws
SRC=$WS/src
BRANCH=main   # or master

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== Rover Update Script ===${NC}"

source /opt/ros/noetic/setup.bash >/dev/null

cd "$SRC"

echo -e "${GREEN}Pulling latest code from Git...${NC}"
for dir in */ ; do
    if [ -d "$dir/.git" ]; then
        echo -e "${GREEN}Updating $dir ...${NC}"
        cd "$dir"
        git fetch origin
        git checkout $BRANCH
        git pull --rebase || git pull
        cd ..
    fi
done

echo -e "${GREEN}Rebuilding...${NC}"
cd "$WS"
catkin_make -DCMAKE_BUILD_TYPE=Release

echo -e "${GREEN}=== Update complete ===${NC}"
