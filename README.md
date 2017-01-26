# The software id developed by TEAM DISHARI
#Team members
# 1. Mrinmoy Sarkar (Algorithm and software developer and ROS Programmer)
# 2. Dhiman Chowdhury (Algorithm developer)
# 3. Md Zakaria Haider (Team Leader)

# HRATC 2016 Codebase for Team Dishari
- http://www2.isr.uc.pt/~embedded/events/HRATC2015/Welcome.html

Unzip the "teamd" code folder to ~/hratc2016_workspace/src/

The instructions below assume you have installed the HRATC 2016 code/simulator at ~/hratc2016_workspace/src

# Procedure to install package assuming teamd code folder is inside ~/hratc2016_workspace/src
- cd ~/hratc2016_workspace
- catkin_make


#############
# Components used
#############

The laser scanner, mine sweep arm, and robot base is used. The laser navigation stack is used.

#############
# FOR Simulation runs on scenario1 gazebo world
#############

# Run the Simulator
- roslaunch hratc2016_framework run_simulation_world.launch
#wait for 10s
open a new terminal
# Run Team Dishari navigation support nodes
- roslaunch teamd teamdNavigation.launch
#wait for 10s
open a new terminal
# Run Team Dishari minedetection support nodes
- roslaunch teamd teamdMinedetector.launch


# if the robot does not move or the program hangs up, please restart the whole system after waiting 2/3 minutes


