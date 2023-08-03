# Turtlebot-movebase-navigation

Turtlebot follows the trajectory of real world in a simulated  environment  
# 7.21 Updata
evo:Python package for the evaluation of odometry and SLAM

```
pip install evo --upgrade --no-binary evo
```

In sim_traj.txt and real_traj.txt,trajectories are  saved as tum format.
```
timestamp x y z q_x q_y q_z q_w
```
 test_evo.py:Align timestamps of sim_traj.txt and real_traj.txt,and analyze the differences between simulated and real trajectories.Open a terminal run:

```
python test_evo.py
```

# Installation
 Install ROS,if it is not already installed.  

 
```
sudo apt install ros-melodic-desktop-full
```   

 
 Add environment variables to bashrc： 

 
```
sudo gedit ~/.bashrc
```  


Add the following content at the end of the file  
```
source /opt/ros/melodic/setup.bash
source ~/turtlebot_workspace/devel/setup.bash
export TURTLEBOT_BASE=kobuki  
export TURTLEBOT_STACKS=hexagons
export TURTLEBOT_3D_SENSOR=hokuyo
```

Refresh environment variables：

```
source ~/.bashrc
```

Apt-install these packages:
```
sudo apt-get install ros-melodic-ecl-exceptions
sudo apt-get install ros-melodic-ecl-threads
sudo apt-get install ros-melodic-joy*
sudo apt-get install ros-melodic-kobuki-driver
sudo apt-get install ros-melodic-ecl-streams
sudo apt-get install ros-melodic-yocs-controllers
sudo apt-get install ros-melodic-depthimage-to-laserscan
sudo apt-get install ros-melodic-yocs-velocity-smoother
sudo apt-get install ros-melodic-gmapping
sudo apt-get install ros-melodic-map-server
sudo apt-get install ros-melodic-navigation
```
Clone this repository on your computer and catkin_make(catkin_make in the turtlebot_workspace directory):
```
mkdir -p turtlebot_workspace/src
cd turtlebot_workspace/src
git clone https://github.com/zmy2017/Turtlebot-path-following.git
cd ..
catkin_make
```
All of the dependencies are in the environment.yml file. They can be installed manually or with the following command:
```
conda env create -f environment.yml
```
# RUN
Here is an example navigation run command.In test_navigation.py，first execute domain randomization，then execute navigation

On terminal 1 launch ROS and gazebo server:
```
cd ~/turtlebot_workspace
roslaunch gazebo_domain_randomizer amcl.launch
```
On terminal 2 run:
```
python test_navigation.py
```

# Additional Remarks
After  manually forcing interruption the terminal where Gazebo located,use the following commands to kill the Zombie process:

```
killall gzserver
killall gzclient
```
# Directory Structure
```
├── environment.yml                           # Conda environment configuration
├── gazebo_domain_randomizer                  # Primary Python module of this project
│   ├── launch                                # Launch directory
│   ├── maps                                  # Maps for rviz
│   ├── scripts                               # Python files
│   │   ├── gazebo_domain_randomization.py    # Set sim/real world env and modify Gazebo parameters(currently include  mass of Turtlebot,friction coefficient of two wheels)
│   │   ├── gazebo_env.py                     # Functions that interact with Gazebo
│   │   ├── get_map.py                        # Select starting and destination points based on the map
│   │   ├── robot_description.urdf            # Currently is useless.Robot description integrated into one file,convenient for reading
│   │   ├── test_domain_randomization.py      # test code for domain randomization
│   │   ├── test_navigation.py                # Example code
│   └── SimpleWorld                           # Worlds for simulation environments
├── gazebo_ext_msgs                           # Custom Service Data
│   └── srv                                   # include GetSurfaceParams,SetSurfaceParams and others custom services' formats
├── gazebo_physics_plugin                     # Physics plugins for implementing custom services
├── gazebo_scene_plugin                       # Scene plugins for implementing custom services(currently not use)
├── requirements.txt                          # Of no use,use environment.yml is enough
└── Turtlebot_profile                         # Turtlebot official configuration file,include robot description files,Movebase for navigation,and amcl for location.


```

