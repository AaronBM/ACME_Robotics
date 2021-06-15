# ACME_Robotics
SAE Robotics BootCamp Summer 2021

============sae_aeb_hainley.py INSTRUCTIONS========================

1) Navigate to your catkin workspace and add remote repository from git
$ git clone https://github.com/AaronBM/ACME_Robotics

2) Refresh catkin.  Move up one directory from toplevel src file and run:
$ catkin_make

3) Make sure that the f1_tenth launch file has been modified to use the wall world.
  In the launch file make sure that "racecar_wall" is uncommented and that all other worlds are commented out.
  
4) Run the launchfile
$ roslaunch f1_tenth.launch

5) Run the script in a separate terminal
$ rosrun ACME_Robotics sae_aeb_hainley.py


------------Metholodoly----------------------------------------------
The algorithm searches for the straight ahead angle and returns the average
distance measured in a 4 degree cone.  This distance is then fed into a 
velocity controller with the following logic.
  1) If the error (current distance - threshold value of 3 m) multiplied by the gain is greater than the max velocity, then command max velocity
  2) If the error multiplied by the gain is less than the max velocity, command a velocity equal to the error multiplied by the gain
  3) If the error multiplied by the gain is negative, command zero velocity.
