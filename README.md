# RTAB-Map Docker Project

This project is a work-in-progress implementation of RTAB-Map (Real-Time Appearance-Based Mapping) using Docker. It provides a flexible and portable solution for SLAM (Simultaneous Localization and Mapping) experiments on both Windows and Ubuntu.

---

## Project Overview

The goal of this project is to simplify the deployment and execution of RTAB-Map in a Dockerized environment, supporting various platforms:

- **Windows:** Uses `xLaunch` for visualization.
- **Ubuntu:** Directly integrates with the X-server.

The project is designed to:

- Build and run RTAB-Map within a Docker container.
- Support USB cameras for SLAM.
- Provide visualization via RViz.

> **Note:** This project is still under development. Features and configurations may change as the work progresses.

---

## Prerequisites

### General Requirements
- **Docker** (version 20.10 or higher)
- **Docker Compose** (version 1.29 or higher)

### For Windows
- [Docker Desktop](https://www.docker.com/products/docker-desktop)
- [xLaunch (VcXsrv)](https://sourceforge.net/projects/vcxsrv/)

### For Ubuntu
- Native Docker installation
- X-server support

---

## Setup and Installation

### Clone the Repository
```bash
git clone <repository-url>
cd rtabmap_project
```

### Configure the Environment
Edit the `.env` file to match your platform:

#### Windows Configuration
```env
DISPLAY=host.docker.internal:0.0
```

#### Ubuntu Configuration
```env
DISPLAY=:0
```

### Build and Run the Container
```bash
docker-compose up --build
```

---

## Usage

### Inside the Container
1. **Check Camera Connection:**
   ```bash
   v4l2-ctl --list-devices
   ```

2. **Launch RTAB-Map:**
   ```bash
   ros2 launch rtabmap_ros rtabmap.launch.py \
     rgb_topic:=/camera/image_raw \
     frame_id:=camera_link
   ```

3. **For USB Cameras Without Depth Data:**
   ```bash
   ros2 launch rtabmap_ros rtabmap.launch.py \
     rgb_topic:=/camera/image_raw \
     frame_id:=camera_link \
     subscribe_depth:=false \
     subscribe_rgbd:=false
   ```

### Visualize in RViz
Run RViz inside the container:
```bash
rviz2
```
Configure RViz to display:
- `/rtabmap/mapData`
- `/rtabmap/odom`
- `/rtabmap/cloud`

---

## Stopping the Container
To stop and clean up the container:
```bash
docker-compose down
```

---

## Known Issues
- Performance may be slower on systems without dedicated GPUs.
- USB camera compatibility may vary depending on drivers.
- Visualization may require additional configuration on Windows (`xLaunch`).

---

## Future Improvements
- Add support for multiple cameras and sensors.
- Optimize for GPU-based systems.
- Improve documentation and examples for SLAM experiments.

---

## Contributing
Feel free to submit issues, feature requests, or pull requests to help improve the project.