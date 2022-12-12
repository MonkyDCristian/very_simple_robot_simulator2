# A Very Simple Robot Simulator ROS2 version.

![Alt text](/imgs/vsrs2.png)

## Dependencies
 
* Operating system: [Ubuntu 22.04](https://releases.ubuntu.com/22.04/)
* Communication Software: [ROS2 Humble (desktop)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* GUI Software: [Pygame](https://www.pygame.org/wiki/GettingStarted)

## Install and Compile
```
mkdir -p ~/<your_ws>/src && cd ~/<your_ws>/src
git clone https://github.com/MonkyDCristian/very_simple_robot_simulator2.git
cd ~/<your_ws> && colcon build
cp -r ~/<your_ws>/src/very_simple_robot_simulator2/launch ~/<your_ws>/install/very_simple_robot_simulator2/share/very_simple_robot_simulator2
echo "source ~/<your_ws>/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Operation Modes
* Idle mode: non interactive interface
* Add Wall mode: press 'w' key to draw a wall on the map with the mouse pointer.
* Delete Wall mode: press 'd' key to delete a wall.
* Set Robot Pose mode: press 'p' key to change the robot pose. Drag and drop the robot to change the position. Click outside the robot to change its orientation.

## Architecture
![Alt text](/imgs/rosgraph.png)




