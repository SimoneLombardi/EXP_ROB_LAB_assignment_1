# EXP_ROB_LAB_assignment_1
## Run the simulation
To test this assignment download the repository in the /src forlder of your workspace. Build the environment, than use: 
```bash
"ros2 launch erl_ass1_pkg gazebo.launch.py"
```
to run the simulation
It is important to have the models of the markers in the /.gazebo/models folder in your machine. You can find the models here: https://github.com/CarmineD8/aruco_ros.git int the folder /aruco_ros/models. 
Copy paste all the folders in the /.gazebo/models folder(you may need to create the /models folder).

Than in a separate cmd, use the command: 

```bash
"ros2 run erl_ass1_pkg routine_interface"
```
to decide if the 'camera' or 'robot' will be rotating. Once all the markers have been visited in order the simulation stops on itselfs.
