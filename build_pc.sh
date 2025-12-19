#!/bin/bash
set -e

# ======================================================================
#  build_pc.sh  Build rover workspace on the main PC (Noetic)
# ======================================================================

WS=~/rover_wse
SRC=$WS/src
LOG="$WS/build_pc.log"

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== Rover Build Script (PC / Simulation + MoveIt) ===${NC}"
echo -e "Log: $LOG"
echo -e "Workspace: $WS"

# ----------------------------------------------------------------------
# 1. Validate workspace
# ----------------------------------------------------------------------
if [ ! -d "$WS/src" ]; then
    echo -e "${RED}ERROR: $WS/src does not exist.${NC}"
    exit 1
fi

# ----------------------------------------------------------------------
# 2. Environment setup
# ----------------------------------------------------------------------
source /opt/ros/noetic/setup.bash || true
[ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash"

# ----------------------------------------------------------------------
# 3. Remove CATKIN_IGNORE files for PC
# ----------------------------------------------------------------------
echo "Unignoring all packages..."
find "$SRC" -name CATKIN_IGNORE -type f -delete

# ----------------------------------------------------------------------
# 4. Install missing dependencies
# ----------------------------------------------------------------------
echo -e "${GREEN}Installing ROS dependencies...${NC}"
rosdep install --from-paths "$SRC" --ignore-src -r -y | tee "$LOG"

# ----------------------------------------------------------------------
# 5. Build workspace
# ----------------------------------------------------------------------
echo -e "${GREEN}Building (full sim + MoveIt)...${NC}"
cd "$WS"
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo 2>&1 | tee -a "$LOG"

echo ""
echo -e "${GREEN}=== Build completed successfully on PC ===${NC}"
