# `recon_sim`
package for simulating the Recycled Conveyor Belt Project
# Installation
**Note**: Please install `libfranka` before proceeding.

Install a few dependencies for building
```
sudo apt update && sudo apt install -y \
	python3-wstool \
	python3-catkin-tools \
	python3-rosdep
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
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
```

To run the demo
```
roslaunch recon_controllers recon_panda.launch controller:=joint_position_controller
```
