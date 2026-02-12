# RoboHaptics Toolkit (Unitree Go2 EDU v1.0.0)

This is the repository that contains the software ecosystem for the paper titled "RoboHaptics:  Designing Haptic Interactions for the Lower Body with
Quadruped Robot Dogs" (ACM CHI 2026).

RoboHaptics is a Python toolkit for authoring **lower-body haptic effects** using a Unitree Go2 EDU quadruped. It wraps the Unitree SDK and exposes posture/effect primitives built from **time-parameterized joint trajectories** (default control loop ~200 Hz).

## Disclaimer / Assumption of Risk

This toolkit is research software provided “as is” and may contain bugs, incomplete safety checks, or other defects. It can actuate real robotic hardware and may cause unexpected motion, property damage, or physical injury if used improperly.

By downloading, installing, or using this code, you acknowledge that you understand these risks and agree that you are solely responsible for safe setup, supervision, operation, and compliance with all applicable rules and regulations. You agree that the authors and contributors are not liable for any claims, damages, losses, or injuries arising from the use or misuse of this software or associated hardware, to the maximum extent permitted by law.

If you do not agree with these terms, do not use this toolkit.

> **Safety note (read first):** This toolkit can actuate a real robot. Use only in a clear area, keep feet/hands away from moving legs, and always be ready to stop the program immediately if the robot behaves unexpectedly.

---

## Hardware / Software Requirements

- **Robot:** Unitree Go2 EDU
- **Network:** robot connected via Ethernet or the Unitree network setup
- **Python:** 3.9+ (recommended 3.10/3.11)
- **Dependencies:** `unitree_sdk2py` and its runtime dependencies

---

## Safety Guidelines

- Use in an open area with clear floor.
- Keep body parts away from leg workspace.
- Start with slow trajectories (t_move longer, conservative gains).
- Always have a fast way to stop the process (keyboard interrupt / kill).
- If behavior is unexpected, stop immediately and re-run initialization before continuing.

---

## Citation

If you use this toolkit in academic work, please cite the RoboHaptics project/paper:

- Add your BibTeX entry here once finalized.

---

## License

This project is released under the **MIT License**. See `LICENSE` for details.

# unitree_sdk2_python
Python interface for unitree sdk2 from Unitree Robotics. Research purpose only, not for commercial use.

# Installation
## Dependencies
- Python >= 3.8
- cyclonedds == 0.10.2
- numpy
- opencv-python
## Install unitree_sdk2_python

```bash
pip install unitree_sdk2py
```

### Installing from source
Execute the following commands in the terminal:
```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/HuanjunZhao/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```
## FAQ
##### 1. Error when `pip3 install -e .`:
```bash
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
```
This error mentions that the cyclonedds path could not be found. First compile and install cyclonedds:

```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```
Enter the unitree_sdk2_python directory, set `CYCLONEDDS_HOME` to the path of the cyclonedds you just compiled, and then install unitree_sdk2_python.
```bash
cd ~/unitree_sdk2_python
export CYCLONEDDS_HOME="~/cyclonedds/install"
pip3 install -e .
```
For details, see: https://pypi.org/project/cyclonedds/#installing-with-pre-built-binaries

# Usage
The Python sdk2 interface maintains consistency with the unitree_sdk2 interface, achieving robot status acquisition and control through request-response or topic subscription/publishing. Example programs are located in the `/example` directory. Before running the examples, configure the robot's network connection as per the instructions in the document at https://support.unitree.com/home/en/developer/Quick_start.
## DDS Communication
In the terminal, execute:
```bash
python3 ./example/helloworld/publisher.py
```
Open a new terminal and execute:
```bash
python3 ./example/helloworld/subscriber.py
```
You will see the data output in the terminal. The data structure transmitted between `publisher.py` and `subscriber.py` is defined in `user_data.py`, and users can define the required data structure as needed.
## High-Level Status and Control
The high-level interface maintains consistency with unitree_sdk2 in terms of data structure and control methods. For detailed information, refer to https://support.unitree.com/home/en/developer/sports_services.
### High-Level Status
Execute the following command in the terminal:
```bash
python3 ./example/high_level/read_highstate.py enp2s0
```
Replace `enp2s0` with the name of the network interface to which the robot is connected,.
### High-Level Control
Execute the following command in the terminal:
```bash
python3 ./example/high_level/sportmode_test.py enp2s0
```
Replace `enp2s0` with the name of the network interface to which the robot is connected. This example program provides several test methods, and you can choose the required tests as follows:
```python
test.StandUpDown() # Stand up and lie down
# test.VelocityMove() # Velocity control
# test.BalanceAttitude() # Attitude control
# test.TrajectoryFollow() # Trajectory tracking
# test.SpecialMotions() # Special motions
```
## Low-Level Status and Control
The low-level interface maintains consistency with unitree_sdk2 in terms of data structure and control methods. For detailed information, refer to https://support.unitree.com/home/en/developer/Basic_services.
### Low-Level Status
Execute the following command in the terminal:
```bash
python3 ./example/low_level/lowlevel_control.py enp2s0
```
Replace `enp2s0` with the name of the network interface to which the robot is connected. The program will output the state of the right front leg hip joint, IMU, and battery voltage.
### Low-Level Motor Control
First, use the app to turn off the high-level motion service (sport_mode) to prevent conflicting instructions.
Execute the following command in the terminal:
```bash
python3 ./example/low_level/lowlevel_control.py enp2s0
```
Replace `enp2s0` with the name of the network interface to which the robot is connected. The left hind leg hip joint will maintain a 0-degree position (for safety, set kp=10, kd=1), and the left hind leg calf joint will continuously output 1Nm of torque.
## Wireless Controller Status
Execute the following command in the terminal:
```bash
python3 ./example/wireless_controller/wireless_controller.py enp2s0
```
Replace `enp2s0` with the name of the network interface to which the robot is connected. The terminal will output the status of each key. For the definition and data structure of the remote control keys, refer to https://support.unitree.com/home/en/developer/Get_remote_control_status.
## Front Camera
Use OpenCV to obtain the front camera (ensure to run on a system with a graphical interface, and press ESC to exit the program):
```bash
python3 ./example/front_camera/camera_opencv.py enp2s0
```
Replace `enp2s0` with the name of the network interface to which the robot is connected.

## Obstacle Avoidance Switch
```bash
python3 ./example/obstacles_avoid_switch/obstacles_avoid_switch.py enp2s0
```
Replace `enp2s0` with the name of the network interface to which the robot is connected. The robot will cycle obstacle avoidance on and off. For details on the obstacle avoidance service, see https://support.unitree.com/home/en/developer/ObstaclesAvoidClient

## Light and volume control
```bash
python3 ./example/vui_client/vui_client_example.py enp2s0
```
Replace `enp2s0` with the name of the network interface to which the robot is connected.T he robot will cycle the volume and light brightness. The interface is detailed at https://support.unitree.com/home/en/developer/VuiClient
