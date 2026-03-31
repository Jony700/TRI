# Developer Guide

The purpose of this file is to guide both users and developers through the process of building, running, and modifying the code in this repository.

Here, we'll be adressing which files are important, and which should not be modified.

## The Environment

Looking at this repository with no ROS2 or Gazebo knowledge might be quite overwhelming due to the ammount of files available, as well as their complex structure and contents.

But fear not! The most important environment files are directly pulled from Gazebo's `andino_gz` package. The `andino_gz` package is a Gazebo simulation of the Andino robot, whihc is just essentially a differential drive robot with a LiDAR sensor. 

*Complete requirements in [README.md](README.md)*

For setup, simply run:

```bash
cd ~
mkdir tri_ws
cd ~/tri_ws
source install/setup.bash
colcon build
mkdir src
```

And clone the repository to `src`!


> [!IMPORTANT]
> **DevLog:** Remember to always build whenever any change is made to the project! Otherwise, commands will use the cached files instead of the new information.



```bash
cd ~/tri_ws
source install/setup.bash
colcon build
```

## The World

Our world was manually created without using Gazebo's GUI, for complete control of every parameter. By following this project structure:


```raw

├── andino_gz/                        
│    └── andino_gz/
│         ├── worlds/
│         │   └── assignment1.sdf   

```


You can find the `.sdf` file, which contains every parameter of the world, more specifically wall settings. Here, you can find every customizable setting, such as their position, size, and collisions. 


> [!IMPORTANT]
> **DevLog:** Since this part of the assignment is complete, there's no need for further development here.


## The Robot

The robot is controlled by a python script, which can be found in:

```raw

├── wall_follower/                    
     ├── wall_follower/
     │   ├── __init__.py
     │   └── wall_follower_node.py      
     ├── launch/
     │   └── wall_follower.launch.py    
     ├── setup.py                       
     └── package.xml                    

```

`wall_follower_node.py` is responsible for the following core functionalities:

1. **Input**: Reading the LiDAR data from the `/scan` topic, creating a bridge between Gazebo and ROS2
2. **Output**: Returns (publishes) velocity commands (which are Twist messages) to drive the robot
     1. Twist is a ROS2 message type that contains the linear and angular velocity of the robot
3. **PID Controller**: Setting up the PID controller (Proportional-Integral-Derivative)
     1. It's called Proportional-Integral-Derivative because it uses the proportional, integral, and derivative of the error to calculate the velocity commands


The driving and wall-following logic also lives here. As of march 19th:

1. `_get_min_range_sector`: checks which LiDAR identifies the closest instance of collision (closest wall)
2. `valid_reading_ratio`: checks percentage of valid LiDAR readings (not NaN, not inf, etc.)
3. `scan_callback`: Checks if the robot is inside of the circle by measuring if at least 60% of the LiDARs detect a wall
     1. Works by making the robot always follow the wall on its left (*Measure Key Sectors*)
     2. `if front_dist < self.front_obs_dist`: Uses average distance of 2 front sensors do assess need for sharp turns to avoid collision (*State machine*)
     3. `elif wall_dist > self.max_search_dist`: If no walls are found, robot walks forward with a slight left steer to find LiDAR collision points (*State machine*)
     4. `else`: Follows the wall on its left by constantly adjusting angular velocity based on `desired_dist = 1.5m`. This also works to avoid hitting walls inside the circle, but requires extra conditions to center and leave as requested in the guidelines. (*State machine*)

> [!IMPORTANT]
> **DevLog:** Terminator is recommended, as at least two simultaneous terminals are required to be running in paralell.


## Launch Settings

`assignment1.launch.py` works sort of like a config file here: it contains multiple functions to help maintain stability and higher control of the project. It features multiple different useful setups, like choosing whether the robot will start in a random or fixed position. 

> [!IMPORTANT]
> **DevLog:** It's not a mandatory read for development, but it's quite useful! 

## TODO

1. Create optional fixed start position (useful for development)
2. Create stop-center-turn-leave condition (requested in guidelines) 
3. Change [[The Robot]](#the-robot) section, João fixed and now the robot is 100% reactive (30/03/2026) - maybe i'll leave it to Rafa since he likes to write in these type of files

## Issues

1. Theoretically, I've added both but none are working...
2. Next implementation will incude debugging:
     1. fixed start might not be getting triggered
     2. centering condition is not working or regular wall follow is not being overridden by center_align

