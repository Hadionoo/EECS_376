## my_lidar_alarm
A simple, illustrative program to show how to subscribe to a LIDAR signal and interpret the signal.
This code subscribes to topic `robot0/laser_0`, which is published by the Simple 2-D Robot simulator.
The signal interpretation in this program looks at multiple pings around the robot to navigate it safely around the maze.  For these pings, if the ping distance is less than some danger threshold, the lidar listener publishers a warning signal on topic `lidar_alarm`.  The distance of each of the pings is also published, on topic `lidar_dist`.

## Example usage
Start up the STDR simulator:
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
Start the lidar alarm node:
 `rosrun my_lidar_alarm my_lidar_alarm`
Start the reactive commander node:
`rosrun my_stdr_control my_reactive_commander`

    
