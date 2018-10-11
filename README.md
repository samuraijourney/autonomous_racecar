## Lab 0
This project contains skeleton code for lab0. The first portion of the lab requires the student to publish poses for a car clone that follows the original car around. The second portion requires the student to record a bag file of the robot driving around while being tele-operated. Then a script is written to play back the bag file such that the robot autonomously follows (approximately) the path that was recorded in the bag. See the [assignment spec](docs/lab0_spec.pdf) for more details about the assignment.

### Installation
The following packages are required:
1. ackermann_msgs: **sudo apt-get install ros-kinetic-ackermann_msgs**
 
2. numpy: **sudo apt-get install python-numpy**

Once these packages have been installed, clone this package, place it in a catkin workspace, and compile using catkin_make (or whichever build mechanism you prefer).
  
### Usage

After implementing CloneFollower.py and CloneFollower.launch, the following command will launch the script for the first portion of the assignment: **roslaunch lab0 CloneFollower.launch**

After implementing BagFollower.py and BagFollower.launch, the following command will launch the script for the second portion of the assignment: **roslaunch lab0 BagFollower.launch**
