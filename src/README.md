# Assisted Navigation

## Prerequisites
- ROS1 Melodic  
## Building
1. To start building, first clone the repository using the following command, specifying the "an_ToF" branch:

```
git clone -b an_ToF https://github.com/Roboy/esp-wheelchair.git
```
2. Next, navigate to the "src" folder within the repository.

3. Copy the "assisted_navigation" package to your workspace.

4. Build the workspace using the "catkin_make" command.

5. Finally, source the setup bash file by running the following command:
```
source /devel/setup.sh
```

With these steps, you should be ready to proceed with the build process.

## Running
1. To execute the sensor, you can use either of the following commands:

```
rosrun roboy_state_estimator ir_sensor_publisher.py
```
or 

```
rosrun assisted_navigation ir_sensor_publisher.py
```

2. These commands will run the "ir_sensor_publisher.py" script, which is responsible for publishing data from the IR sensor.

3. To launch the middleware, you can use the following command:

```
rosrun assisted_navigation ir_drive_controller.py
```
This will start the middleware module, allowing it to handle the communication and processing of data between different components of the system.

With these commands, you should be able to run the sensor and middleware components of the system as needed.

## Teleop_twist_keyboard
1. Make sure Teleop_twist_keyboard is installed

2. make sure node Ir_drive_controller is running


3. run twist_to_pwm node using the following command 
```
rosrun assisted_navigation twist_to_pwm.py
```

4. In another terminal run the teleop node using the following command 

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Use the instruction in the terminal

## Simulation
1. Make sure Gazebo is installed

2. Make sure USE_SIMULATION is TRUE in the src/ir_drive_controller.py 
*if its false then set to true then redo all the building steps

```
USE_SIMULATION = True 
```

3. Run Gazebo using the following command
```
gazebo
```
4. Spawn the robot using the following command

```
roslaunch assisted_navigation spawn.launch
```

Use the teleop or joystick to control the robot

 

