# DroneQuest

## Purpose
DroneQuest is a multiplayer game where players complete the "quest" by **flying real drones using their hands gestures** to hover over randomly-chosen destinations. This work was developed in NIMBUS Lab under NSF Grant CNS-2244116 to be used as a demonstration module for STEM outreach. 

<p align="center">
  <img src="[demo.gif](https://raw.githubusercontent.com/radall1/DroneQuest/main/frames/demo2.gif)" />
</p>

![demo](https://raw.githubusercontent.com/radall1/DroneQuest/main/frames/demo.jpg)

## Installation
The following steps have been verified to work on Ubuntu Linux 20.04 (you can check your version by typing `lsb_release -a` in the terminal) with ROS 1. Your experience may vary with other versions of Ubuntu and/or ROS. 

### Dependencies 
Install the following dependencies using `pip3`:
```sh
pip3 install mediapipe==0.8.1
pip3 install opencv-python==4.8.0
pip3 install tensorflow==2.3.0
pip3 install numpy==1.18.5
```

### File Preparation
1. Clone the repository to your Linux device 
```sh
git clone https://github.com/radall1/DroneQuest.git
```
2. Edit the 3rd and 4th lines in the `DQ.desktop` file to point to the correct path of `launch_dronequest.sh` and `icon.jpg`
```desktop
Exec=/home/reustudents/DroneQuest/launch_dronequest.sh
Icon=/home/reustudents/DroneQuest/icon.jpg
```
3. Edit the 4th line in the `launch_dronequest.sh` file to point to the correct path of the `DroneQuest` folder
```desktop
# Change the directory to the script's location
cd /home/reustudents/DroneQuest/
```
4. Move the `DQ.desktop` launcher file to your Desktop.
```sh
mv ~/[insert_your_path]/DroneQuest/DQ.desktop ~/Desktop
```
5. Run `catkin_make` or `catkin build` inside the `DroneQuest` directory.
```sh
cd ~/[insert_your_path]/DroneQuest 
catkin build
```

### Drone Preparation
The drone must be registered on Vicon (motion capturing system) and compatible with the open-source Freyja (precise control flight stack) with the following caveats:

1. When calibrating Vicon, the "north" of the drone should be the "west" of the player. In NIMBUS Lab, this means that the "north" of the drone should be pointing towards the two rows of desks. This is critical. 
2. The default name for the drone in this code is `GREG`. Once you create an object for your drone in Vicon, you should change `GREG` to the name you chose for your drone object in two locations: line 67 in `game.py` and line 12 in `freyja_controller.launch`.
```python
self.subscriber = rospy.Subscriber("/vicon/GREG/GREG", TransformStamped, self.drone_data_callback)
```
```xml
 <arg name="vicon_topic"               default="/vicon/GREG/GREG"/>
```
3. Freyja is typically used with custom firmware (hosted on the NIMBUS Drive). However, it can be used with new versions of ArduPilot (4.1+). For this to work, you need to (i) use the flight mode `Guided_NoGPS` for Freyja to "take over" and (ii) set ThrustAsThrust field for the parameter `G2_OPTIONS`.

### Launch
To launch the game, double click on the `DroneQuest` desktop application that should be located on your Desktop. Alternatively, you could run the `LAUNCH_ME.py` file in your `DroneQuest` directory using the following command:
```sh
python3 ~/[insert_your_path]/DroneQuest/LAUNCH_ME.py
```
