# Simulation, and Control of a 3-DOF Coaxial Spherical Parallel Manipulator

Project involves joystick control of CSPM simulation in CoppeliaSim. The trajectory can be then replayed on physical model using Dynamixel Workbench. Qt GUI is connected to simplify control. Coaxial optimization of the control in CoppeliaSim simulation is also attempted. 
Project is implemented in ROS.

Software used in the project:
1. CoppeliaSim simulator - https://www.coppeliarobotics.com/helpFiles/en/ros1Tutorial.htm - full tutorial on downloading and installing the software. CoppeliaSim can be downloaded anywhere. simExtROS package needs to be downloaded and placed in catkin workspace. 
To use: Initiate CoppeliaSim with terminal from downloaded folder. Open CSPM model in CoppeliaSim. 

2. Dynamixel Workbench - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/ 
To use: Launch dynamixel_controllers.launch file and run phys_model.cpp node to connect to dynamixel motors of physical model.

roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch
rosrun sim_pkg phys_model

3. Qt GUI - install file is included for GUI with ROS plugin. Packages to include in workspaces are also included. (optional)
To use: Open Qt creator and choose ROS workspace (with the package) from other projects. Run the project to open the GUI window.

4. CVXGEN - https://cvxgen.com/docs/index.html. All required files are included in package source. (optional)

Project can be used with different features:

1. Simple control with separate node for every mode

Joystick control is run from joy_v1.cpp node. With connected DS4 joysitck run the cpp file.
To use cvxgen, run testsolver.cpp. Same code as joy_v1 with additinal functionality of optimizer. 
Trajectory outputs, joint angles, are recorded in rosbag.

Move to configuration is run from move_to_config.cpp node. Joint values are expected from /joints_mtc topic. (If used separately, rostopic pub /joints_mtc with values needs to be published.)

Repetition of recorded trajectory. Output values are read from rosbag file and run either in simulation with rec.cpp or in physical model with rec_v2.cpp.

2. Combined control with modes in one file and GUI (or by sending keywords to /hello_gui_node/gui_msgs)

Same control modes in one file. Expects keyword input to choose the control mode ("joystick" for joystick control, "mtc" for move to configuration, "recorded_sim" and "recorded" for repeating recorded trajectory in simulation and physical model respectively). After finishing, returns to idle waiting state. 
In the move to configuration mode, joint values are expected from /hello_gui_node/chatter topic rather than /joints_mtc.

