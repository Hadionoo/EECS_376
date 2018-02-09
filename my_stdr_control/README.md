## my_stdr_control

## my_stdr_open_loop_commander
A node that control the STDR Mobile Robot with open loop commands to traverse the maze and reach the top left corner of the maze without hitting the walls. 

## Example usage
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
to start the simulator.  Run a simple, open-loop command sequence with:
`rosrun stdr_control my_stdr_open_loop_commander`

## my_reactive_commander
A node that controls the STDR Mobile Robot by subscribing to the `lidar_alarm` topic and stopping and turning the robot CCW while `lidar_alarm` is publishing true which means that the robot is close to an obstacle. 

## Example usage
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
to start the simulator
`rosrun my_lidar_alarm my_lidar_alarm`
to send warnings and change the value being published to the lidar_alarm topic
`rosrun my_stdr_control my_reactive_commander`
to make the robot react to warnings 
