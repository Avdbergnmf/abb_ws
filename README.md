# abb_ws
Linking an ABB robot to a Geomagic Touch haptic device using ROS

This is all the contents of the src folder.

  The geomagic touch packages are edited versions of the original omni packages (LINK)
  The ABB ros packages are stripped down versions of the original (LINK)

The pathmaker_abb package is custom written by me.

---Install---

clone this git:
  $ git clone git clone https://github.com/ros-industrial/industrial_core.git
Get the ros industrial core installed:
  $ git clone https://github.com/ros-industrial/industrial_core.git

If you get error: "/usr/bin/ld: cannot find "-lncurses"", you're missing the 32bit libraries. To install them: 
  $ sudo apt-get install lib32ncurses5-dev
  $ rosdep update
  $ rosdep install --from-paths ~/<catkin_ws>/src --ignore-src 

Might need to build twice, while sourcing in between. Might remove this bug later.

To use the geomagic touch package you need the openhaptics and the haptic device drivers by 3ds systems:
https://3dssupport.microsoftcrmportals.com/knowledgebase/article/KA-03284/en-us 
If the first 3 joints remain 0, add this line to your bashrc:
$ export LC_NUMERIC=en_US.UTF-8
Then reconnect through /opt/geomagic_touch_dveice_driver/Geomagic_Touch_Setup

---Launching---
First launch the geomagic touch package:
$ roslaunch geomagic_touch_m geomagic_touch_m.launch 
Then launch the download interface, where the ip is the ip of your ABB robot.
$ roslaunch abb_irb1200_support robot_interface_download_irb1200_5_90.launch robot_ip:=192.168.125.1
Then launch the moveit planner. This launches the required move_group and opens up rviz with the robot model loaded so you can test some initial movements:
$ roslaunch abb_irb1200_5_90_moveit_config moveit_planning_execution.launch
--> to get the target_frame TF frame to show up in rviz (to see where you're moving the robot) click add->TF, then on the left panel browse to TF->frames-> and select the frames you want to see. I usually disable all, so that when starting the last node, only the target_frame is shown.
Now to launch the last node to link the ABB to the geomagic touch:
$ rosrun pathmaker_abb pathmaker_abb

---Usage---
On the geomagic touch press the white button to initially enable the link between the 2 devices. This will make the target_frame appear in the rviz scene with the ABB robot in there (if you followed all steps correctly). 
The white button toggles the connection on/off
The grey button disables the connection while the button is held and also resets the target_frame to match the current abb frame.



