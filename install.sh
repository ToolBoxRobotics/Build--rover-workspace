#!/usr/bin/env bash
set -e

# ============================================================
# install.sh ? One-shot setup for the Raspberry Pi rover SBC
# ============================================================

USER=pi
HOME=/home/$USER
WS=$HOME/catkin_ws
SRC=$WS/src
REPO_URL="https://github.com/YOUR_ORG/rover_ws.git"  # <-- CHANGE THIS
ROS_DIST=noetic

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== Rover Pi Install Script ===${NC}"

if [ "$EUID" -ne 0 ]; then
  echo -e "${RED}Please run as root: sudo ./install.sh${NC}"
  exit 1
fi

# ------------------- 1. Basic packages ----------------------
echo -e "${GREEN}[1/7] Installing base packages...${NC}"
apt update
apt install -y \
  curl git build-essential cmake python3-pip \
  python3-rosdep python3-rosinstall-generator \
  python3-vcstool python3-catkin-tools \
  net-tools

# ------------------- 2. ROS Noetic --------------------------
echo -e "${GREEN}[2/7] Installing ROS Noetic...${NC}"

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros1-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

apt update
apt install -y ros-noetic-ros-base

# Desktop extras optional:
# apt install -y ros-noetic-desktop-full

# ------------------- 3. rosdep init -------------------------
echo -e "${GREEN}[3/7] Initializing rosdep...${NC}"
rosdep init || true
sudo -u $USER rosdep update

# ------------------- 4. Workspace + clone -------------------
echo -e "${GREEN}[4/7] Creating catkin_ws and cloning repo...${NC}"
sudo -u $USER mkdir -p "$SRC"
cd "$SRC"

if [ ! -d "$SRC/.git" ]; then
  sudo -u $USER git clone "$REPO_URL" .
else
  echo "Git repo already present in $SRC, skipping clone."
fi

# ------------------- 5. Install ROS dependencies ------------
echo -e "${GREEN}[5/7] Installing ROS dependencies...${NC}"
sudo -u $USER rosdep install --from-paths "$SRC" --ignore-src -r -y

# ------------------- 6. Build workspace ---------------------
echo -e "${GREEN}[6/7] Building workspace...${NC}"
source /opt/ros/$ROS_DIST/setup.bash
cd "$WS"
sudo -u $USER catkin_make -DCMAKE_BUILD_TYPE=Release

# ------------------- 7. .bashrc & environment ---------------
echo -e "${GREEN}[7/7] Updating ~/.bashrc...${NC}"
BASHRC=$HOME/.bashrc

add_line() {
  grep -qxF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"
}

add_line "source /opt/ros/$ROS_DIST/setup.bash"
add_line "source \$HOME/catkin_ws/devel/setup.bash"
add_line 'export ROS_MASTER_URI=http://PC_IP:11311           # <-- CHANGE PC_IP'
add_line 'export ROS_HOSTNAME=ROVER_IP                       # <-- CHANGE ROVER_IP'

# Optional helper aliases
add_line 'alias rc="roscore"'
add_line 'alias rlog="roslaunch rover_bringup master_hw.launch"'

echo -e "${GREEN}Install complete. Reboot recommended.${NC}"
echo -e "Run: ${GREEN}sudo reboot${NC}"

# After installation, edit ~/.bashrc to set PC_IP and ROVER_IP.
