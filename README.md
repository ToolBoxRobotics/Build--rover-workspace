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
