# `recon_sim`
package for simulating the Recycled Conveyor Belt Project
# Installation
**Note**: Please install `libfranka` before proceeding.

Update `librealsense2` to newest version
```
sudo apt update
sudo apt install ros-melodic-librealsense2
```

It is also highly recommended to change the mass of the realsense to be very close to 0.

Install a few dependencies for building
```
sudo apt update && sudo apt install -y \
	python-wstool \
	python-catkin-tools \
	python-rosdep
```

Install `opencv` (and contrib or ARUCO detection) if you haven't done so
```
python -m pip install opencv-python opencv-contrib-python
```

Pull all the required repo
```
cd /path/to/your_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/tinsirius/recon_sim/master/recon_sim.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys libfranka
```

Then build your workspace

```
catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
```

To run the demo
```
roslaunch recon_sim recon.launch
rosrun recon_sim visual_control.py
```

To run the dual panda demo
```
roslaunch recon_sim recon.launch robot_ns:=panda_1 arm_id:=panda_1 conveyor_length:=3 y:=-0.5
# wait a bit
# pause gazebo
roslaunch recon_sim recon_dual.launch robot_ns:=panda_2 arm_id:=panda_2 y:=0.5
rosrun recon_sim visual_control.py --armID panda_1
rosrun recon_sim visual_control.py --armID panda_2
```
