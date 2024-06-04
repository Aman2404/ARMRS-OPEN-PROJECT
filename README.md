# ARMRS-OPEN-PROJECT
3D MAPPING USING MULTI-ROBOT SYSTEM


### Setup Instructions

Follow these steps to clone the repository and set up TurtleBot3.
### step 1: Clone the Repository

Open a terminal.
Navigate to the directory where you want to clone the repository.
Run the following command to clone the repository:

    git clone https://github.com/Aman2404/ARMRS-OPEN-PROJECT.git

### Step 2: Install TurtleBot3

Ensure you are inside the repository directory (the one you cloned).
Navigate to the src directory:
cd src

Clone the below mentioned TurtleBot3 repository inside the src directory:

    git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git


### Step 3: Copy World and Model Files

Copy world files from my_package to turtlebot_simulations/turtlebot3_gazebo/world:

    cp -r path/to/my_package/worlds/* path/to/turtlebot_simulations/turtlebot3_gazebo/worlds/

Copy model files from my_package to turtlebot_simulations/turtlebot3_gazebo/models:

    cp -r path/to/my_package/models/* path/to/turtlebot_simulations/turtlebot3_gazebo/models/

Replace path/to/my_package and path/to/turtlebot_simulations with the actual paths to your packages.

once you are done with this perform colcon build inside your workspace.

### Step 4: Launch the Simulation with Multiple Robots and mapping scripts
Before launching launch file run this in terminal 

    export TURTLEBOT3_MODEL=waffle

Run the following command to launch the simulation with multiple robots:

    ros2 launch my_package multi_robots.launch.py

Navigate to the directory containing your Python scripts:

    cd ARMRS_OPEN_WS/src/my_package/scripts/

Run the robot_twist.py script:

    python3 robot_twist.py

Run the image_saver.py script:

    python3 image_saver.py

Run the 3d_mapping.py script:

    python3 3d_mapping.py

Note: We are doing 3D reconstruction from a monocular camera; it will take a lot of time on a local laptop based on images gathered from the environment.
    
