# DroneQuest

## Purpose
DroneQuest is a multiplayer game where players complete the "quest" by **flying real drones using their hands gestures** to hover over randomly-chosen destinations. This work was developed in NIMBUS Lab under NSF Grant CNS-2244116 to be used as a demonstration module for STEM outreach. 

<p align="center">
  <img src="https://raw.githubusercontent.com/radall1/DroneQuest/main/frames/demo2.gif" />
</p>

## Game Rules
Each player is given a set of three random destinations to go over. Each succeeding destination is only revealed after the player reaches the preceding destination. The player who reaches their three destinations in the shortest amount of time wins the game. The game user-interface has supporting instructions, below is the starting screen of the game.

![demo](https://raw.githubusercontent.com/radall1/DroneQuest/main/frames/image_1.png)

## Architecture
An overview of the architecture can be seen below:
- **Camera**: any commonly-available 2D camera (such as a webcam) can be used to live-stream the player's hand gesture to the hand gesture recognition unit.
- **Hand Gesture Recognition Unit**: creates a 21-point handmark of the player's hand and then uses the coordinates of the 21 points to identify the gesture by using a neural network model.
- **Drone Remote Control Unit**: receives the identified gesture and sends the corresponding command (e.g. move 30cm to the right) to the drone through radio waves.
- **Game Unit**: this keeps track of the location of the drone from the Vicon cameras and uses this information to determine which of the players won the game (game objective: reach all the assigned spots in the shortest possible time). 
![demo](https://raw.githubusercontent.com/radall1/DroneQuest/main/frames/architecture.png)


## Wiki 
Please refer to [the wiki](https://github.com/radall1/DroneQuest/wiki) for more information on DroneQuest (e.g. how to install and launch it). 
