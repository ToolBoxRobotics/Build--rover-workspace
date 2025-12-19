# Build--rover-workspace


## 1. install.sh ? full Pi bootstrap

Pi runs Ubuntu Server 20.04 

- User: pi
- Workspace: /home/pi/rover_ws
- Your code is in a single git repo with all packages: https://github.com/YOUR_ORG/rover_ws.git (change this)

Save as /home/pi/install.sh and run once:



After installation, edit ``~/.bashrc`` to set PC_IP and ROVER_IP.

#
## 2. systemd services

- User: pi
- Workspace: /home/pi/rover_ws
- ROS: /opt/ros/noetic
- You want services to start after network and on boot.

Copy each unit into /etc/systemd/system/ and sudo systemctl enable ....


#### 2.1 rover_core.service ? main rover stack (master_hw.launch)

``/etc/systemd/system/rover_core.service``


#### 2.2 rover_web_dashboard.service ? static React/HTML dashboard

``/etc/systemd/system/rover_web_dashboard.service``


#### 2.3 rover_teleop.service ? permanent teleop node(s)

``/etc/systemd/system/rover_teleop.service``


Enable & start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable rover_core.service rover_web_dashboard.service rover_teleop.service
sudo systemctl start rover_core.service rover_web_dashboard.service rover_teleop.service
```

#
## 4. GitHub Actions CI: build catkin_ws on every push
This workflow:

Runs on ubuntu-20.04

Installs ROS Noetic

Sets up catkin_ws

Runs catkin_make

Fails if build fails

Create .github/workflows/ros-ci.yml in your repo:

name: ROS Noetic Catkin Build

on:
  push:
    branches: [ main, master ]
  pull_request:
    branches: [ main, master ]

jobs:
  build:
    runs-on: ubuntu-20.04

    env:
      ROS_DISTRO: noetic
      CATKIN_WS: /home/runner/work/catkin_ws

    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Setup ROS repository
        run: |
          sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list'
          curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
          sudo apt-get update

      - name: Install ROS and tools
        run: |
          sudo apt-get install -y ros-$ROS_DISTRO-desktop-full
          sudo apt-get install -y python3-rosdep python3-catkin-tools
          sudo rosdep init || true
          rosdep update

      - name: Create catkin workspace
        run: |
          mkdir -p $CATKIN_WS/src
          # Assuming your repo is structured so that root holds all packages:
          cp -R $(pwd)/* $CATKIN_WS/src/
          ls -R $CATKIN_WS/src

      - name: Install package dependencies
        run: |
          source /opt/ros/$ROS_DISTRO/setup.bash
          cd $CATKIN_WS
          rosdep install --from-paths src --ignore-src -r -y

      - name: Build workspace
        run: |
          source /opt/ros/$ROS_DISTRO/setup.bash
          cd $CATKIN_WS
          catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo

If your repo layout is different (e.g., src inside the repo), adjust the cp line.

## 5. How to Wire It All Together
On the Pi (fresh image):

sudo ./install.sh

copy systemd units into /etc/systemd/system/

sudo systemctl enable rover_core rover_web_dashboard rover_teleop

sudo reboot

On the PC:

Clone repo into ~/catkin_ws

bash build_pc.sh

Use Gazebo/RViz, React dashboard, etc.

GitHub:

Push repo with .github/workflows/ros-ci.yml

CI builds on each push / PR

Docker:

Build rover_pc_build and/or rover_pi_build images for reproducible builds or dev containers.

If you?d like, next I can:

Generate systemd units for the PC (e.g., auto-start Gazebo sim or RViz at boot)

Add log rotation and journalctl convenience scripts

Add a deployment script that SSHes into the Pi, pulls latest code, rebuilds, restarts services in one go.

You said:
Finalize and create the complete catkin_ws folders for the SBC and catkin_ws for the PC for download.
ChatGPT said:
Your downloadable catkin workspace templates for both the Raspberry Pi SBC and the PC are ready.

These include:

Full folder structure

Placeholder CMakeLists.txt

Placeholder package.xml

Matching package names for everything we designed

Pi version excludes simulation/MoveIt

PC version includes simulation + MoveIt packages

? Download Your Workspaces
? SBC (Raspberry Pi CM4) catkin workspace
Includes:

rover_bringup

rover_base

rover_navigation

rover_perception

rover_description

rover_arm

? PC (Simulation + MoveIt) catkin workspace
Includes everything above plus:

rover_simulation

rover_moveit


 SBC side
Full bringup node

rosserial configs

IMU, GPS, INA219 drivers

Ackermann controller

Watchdog

Diagnostics

Teleop

Hardware launch files

URDF & TF tree

? PC side
Full simulation world

Gazebo plugins

MoveIt configs

Navigation stack (Nav2-style)

Behavior tree autonomy scripts

Kinect/OpenCV pipeline

RViz config

ros3djs models
