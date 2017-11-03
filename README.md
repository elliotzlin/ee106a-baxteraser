# ee106a-baxteraser
Baxter will erase your whiteboards

## Dependencies (aka what we're using for our workstation)
* Ubuntu 14.04.5 LTS
* ROS Indigo Igloo
* Baxter robot

I will defer to other resources regarding installing the above. I can only guarantee running the steps below will work, granted you are working in Cory 111, UC Berkeley.

## Setup
In your ```.bashrc```, add the following
1. ```source /opt/ros/indigo/setup.bash```
    * This runs a ROS-specific configuration script every time you open a new terminal which sets several environment variables that tell the system whre the ROS installation is located.
2. ```source /scratch/shared/baxter_ws/devel/setup.bash```
    * This allows us to use packages from the Baxter SDK, which in our case located in the ```/scratch/shared``` directory.
3. ```export ROS_HOSTNAME=$(hostname --short).local```
    * Honestly, I'm not entirely sure why we do this. I assume this is so Baxter knows who it's talking to.

Make sure you re-source your ```.bashrc``` upon making this changes. Now, onto building the project!

## Build
1.. In the root directory, run ```catkin\_make```
