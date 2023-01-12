
What is this about?

This work focuses on using a Thorvald robot in a simulation environment to count the number of grape bunches in a vineyard. Duplication of counts which is main issue is primarily avoided by capturing the images of grapes bunches only once between every set of beacons. It is achieved by using topological navigation method, where every topological node is being setup between the beacons as a parking station to take images. Further duplication of counts due to detection of grape bunch on both the sides is avoided by taking the 3d dimensions of the bunches. In addition to that, this work acheives a decent obstacle avoidance strategy using A* algorithm with appropriate costmap values.


To run the package:


1. Create a workspace folder: mkdir catkin_ws
2. Mount and create a new source folder into it
   "cd catkin_ws", "mkdir src"
3. Move the project_thorvald package inside the src folder or perform "git clone https://github.com/rufussam/Project_thorvald.git" in the terminal
4. Execute "catkin_make" from /catkin_ws
5. Now please source it to this package by giving "cd devel" and "source setup.bash"
6. Perform "roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small"
7. Execute "roslaunch project_thorvald topo_nav.launch" in another terminal
8. Open the topological map visualisation config for RVIZ in project_thorvald/config/	
topo_nav.rviz
9. Execute the "grapes_bunch_detector.py" under /catkin_ws/src/project_thorvald/scripts


To test with the obstacle:

Execute the below in different terminals after source them as instructed the previous execution
1. "roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small launch_move_base:=false"
2. "roslaunch project_thorvald move_base.launch"
3. "roslaunch project_thorvald topo_nav.launch"
4. Place a obstacle on the path to test
5. Run the python script "grapes_bunch_detector.py" under script folder

