# Build--rover-workspace







install.sh ? bootstrap a fresh Pi into a working rover

rover_core.service, rover_web_dashboard.service, rover_teleop.service ? systemd units

Dockerfiles for PC + Pi builds

GitHub Actions workflow to build catkin_ws on every push

You?ll just need to adjust usernames, IPs, and repo URLs.
1. install.sh ? full Pi bootstrap
Assumptions:

Pi runs Ubuntu 20.04 or a ROS-Noetic-compatible OS

User: pi

Workspace: /home/pi/catkin_ws

Your code is in a single git repo with all packages: https://github.com/YOUR_ORG/rover_ws.git (change this)

Save as /home/pi/install.sh and run once:
