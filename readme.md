## ENPM662 Final Project

|Team Members
|--
|Aashrita Chemakura
|Saketh Narayan Banagiri



## Contents
1. Part Files and Assembly
2. Package


## How to run the code
--> Create a catkin_ws and build it, then source it

--> Download the package

--> extract ur5v9.zip in the package folder in the source directory

--> Build and source the workspace

--> Commands to run (open separate terminals and run in the same order)

  roslaunch ur5v9 new_world.launch
  
  rosrun ur5v9 pub.py

  rosrun ur5v9 sub.py

  for teleop:

  roslaunch ur5v9 ur5_urdf.launch (or you can also use new_world.launch)

  rosrun ur5v9 ur5_teleop.py

You can check out the final videos here 
https://drive.google.com/drive/folders/142R0z5HkB-uMKbHJQ2LbY1tbjut2xsBS?usp=sharing

