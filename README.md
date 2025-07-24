# Gesture Drone Control

A ROS2-based system for controlling drones using hand gesture recognition. This project combines computer vision, machine learning, and ROS2 to enable intuitive gesture-based flight commands.

## Features

- Real-time hand gesture recognition
- Drone command mapping from gestures
- PX4 SITL simulation support
- Gazebo simulation environment
- ROS2 integration for modular architecture
- Data collection and model training capabilities

## Prerequisites

- Ubuntu 22.04
- ROS2 (Humble)
- Python 3.8+
- Git
- PX4-Autopilot
- Gazebo (classic)
- MavSDK
- pip dependencies listed below

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/parikshit-06/GestureNav.git
cd GestureNav
```

### 2. Install ROS2 Dependencies

```bash
# Install ROS2 packages
sudo apt update
sudo apt install -y ros-humble-desktop

# Install Python dependencies
pip install -r requirements.txt
```

### 3. Install PX4 Autopilot

```bash
# Clone PX4 Autopilot
cd ~/
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Install dependencies
bash ./Tools/setup/ubuntu.sh

# Build PX4 for simulation
make px4_sitl gz_x500
```

### 4. Install MAVSDK

```bash
# Install build dependencies
sudo apt update
sudo apt install git cmake build-essential libprotobuf-dev protobuf-compiler

# Clone the MAVSDK repository with submodules
git clone https://github.com/mavlink/MAVSDK.git
cd MAVSDK
git submodule update --init --recursive

# Create a build directory and configure the build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=OFF \
      -DBUILD_MAVSDK_SERVER=ON \
      -Bbuild/default -H.

# Complile the server
cmake --build build/default -j$(nproc)

# Install mavsdk_server to /usr/local/bin
sudo cp <correct-path> /usr/local/bin/mavsdk_server
sudo chmod +x /usr/local/bin/mavsdk_server
mavsdk_server udpin://0.0.0.0:14540

```

### 5. Install Gazebo Classic

```bash
# Install Gazebo Classic
sudo apt install gazebo11 libgazebo11-dev

# Install Gazebo ROS packages
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-gazebo-ros-control
```

### 6. Build the ROS2 Workspace

```bash
cd ~/GestureNav
colcon build --packages-select ros2_ws
source install/setup.bash
```

## Configuration

### Environment Setup

Add the following to your `~/.bashrc`:

```bash
# ROS2 Environment
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/GestureNav/install/setup.bash

# PX4 Environment
export PX4_HOME=~/PX4-Autopilot
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
```

## Usage

### 1. Data Collection (Optional)

If you want to train your own gesture model:

```bash
# Navigate to script directory
cd ros2_ws/src/gesture_drone_control/script

# Run data collection
python extracting_data.py
```

### 2. Model Training (Optional)

```bash
# Train the gesture classifier
python train_model.py
```

### 3. Running the Simulation

#### Terminal 1: Start PX4 SITL with Gazebo

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

#### Terminal 2 & 3: Launch ROS2 Nodes

```bash
cd ~/GestureNav/ros2_ws
colcon build
source install/setup.bash

# Launch the gesture control system
ros2 run gesture_drone_control gesture_publisher
ros2 run gesture_drone_control drone_command_node
```

#### Terminal 4: Launch MavSDK protocol

```bash
mavsdk_server udpin://0.0.0.0:14540
```

## Gesture Commands

```bash
# Run visualize.py for gesture commands
python visualize.py
```

## Project Structure

```
GestureNav/
├── asl_dataset/              # Dataset for training
├── data/                     # Collected gesture data
├── dataset/                  # Processed datasets
├── models/                   # Trained ML models
│   ├── gesture_classifier.pkl
│   └── labels.json
├── ros2_ws/                  # ROS2 workspace
│   └── src/gesture_drone_control/
│       ├── gesture_drone_control/
│       |   ├── __init__.py
│       |   ├── drone_command_node.py
│       |   └── gesture_publisher.py
│       ├── models/
|       ├── resource/
|       ├── setup.cfg  
|       └── setup.py
├── script/
│   ├── extracting_data.py
│   ├── realtime_inference.py
│   ├── train_model.py
│   └── visualize.py
├── requirements.txt
└── README.md
```

## Troubleshooting

### Common Issues

1. **Camera not detected**:
   ```bash
   sudo usermod -a -G video $USER
   # Logout and login again
   ```

2. **PX4 SITL fails to start**:
   ```bash
   # Clean build and retry
   cd ~/PX4-Autopilot
   make clean
   make px4_sitl_default gazebo-classic
   ```

3. **ROS2 nodes not communicating**:
   ```bash
   # Check ROS2 domain
   export ROS_DOMAIN_ID=0
   ros2 topic list
   ```

4. **MAVSDK connection issues**:
   ```bash
   # Verify PX4 is running and accepting connections
   netstat -tulpn | grep 14540
   ```

### Performance Optimization

- Ensure adequate lighting for gesture recognition
- Position camera 1 meters from user for optimal detection
- Close unnecessary applications to free up CPU resources
- Use a dedicated GPU if available for faster inference

## Development

### Adding New Gestures

1. Collect data using `extracting_data.py`
2. Retrain the model using `train_model.py`
3. Test with `realtime_inference.py`

### Extending Drone Commands

Modify `drone_command_node.py` to add new flight behaviors and update the gesture mapping accordingly.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

MIT Licensed

## Acknowledgments

- PX4 Development Team
- MAVSDK Community
- OpenCV Contributors
- ROS2 Community

## Support

For issues and questions:
- Create an issue on GitHub
- Check the troubleshooting section
- Review PX4 and ROS2 documentation

---

**Note**: This project is for research and educational purposes. Always follow local regulations and safety guidelines when operating drones.
