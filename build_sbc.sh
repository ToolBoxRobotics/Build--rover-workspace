#!/bin/bash
set -e

# ======================================================================
#  build_pi.sh ? Build rover workspace on Raspberry Pi Save as:
# ~/catkin_ws/build_pi.sh
# (Or in /usr/local/bin/)
# ======================================================================

WS=~/rover_ws
SRC=$WS/src
LOG="$WS/build_pi.log"

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== Rover Build Script (Raspberry Pi CM4) ===${NC}"
echo -e "Log file: $LOG"
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
echo "Sourcing workspace (if exists)..."
[ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash"


# ----------------------------------------------------------------------
# 3. Ignore heavy simulation packages on Pi
# ----------------------------------------------------------------------
echo "Marking PC-only packages as ignored (if present)..."

IGNORE_LIST=(
  "rover_simulation"
  "rover_moveit"
)

for pkg in "${IGNORE_LIST[@]}"; do
    if [ -d "$SRC/$pkg" ]; then
        touch "$SRC/$pkg/CATKIN_IGNORE"
        echo " - Ignored: $pkg"
    fi
done

echo ""

# ----------------------------------------------------------------------
# 4. Install missing ROS dependencies
# ----------------------------------------------------------------------
echo -e "${GREEN}Installing ROS package dependencies...${NC}"
rosdep install --from-paths "$SRC" --ignore-src -r -y | tee "$LOG"



# ----------------------------------------------------------------------
# 5. Build workspace
# ----------------------------------------------------------------------
echo -e "${GREEN}Building workspace...${NC}"
cd "$WS"
catkin_make -DCMAKE_BUILD_TYPE=Release 2>&1 | tee -a "$LOG"

echo ""
echo -e "${GREEN}=== Build completed successfully on Pi SBC ===${NC}"
? 2. build_pc.sh ? PC simulation + MoveIt build script
Save as:
~/rover_ws/build_pc.sh

#!/bin/bash
set -e


