# Pure Pursuit

Pure Pursuit is a map based algorithm, in which which the car picks a preprocessed waypoint to follow based on a lookahead distance and calculates the necessary angular velocity to reach that point. This usually results in a swervy motion in the car unless it is heavily optimized. To generate our waypoints for a track, we first teleoperate the car through the track, generating a map for the track using a SLAM library. We can then either manually pick out waypoints in rviz or run a script that attempts to determine the optimal trajectory through the map, and then finally process it so that the waypoints are evenly distributed. Shown below is our pure pursuit implementation running during one of the time trials held during the semester. This placed us 2nd in the class out of 7 teams. 

![f1tenth](https://github.com/Jixi123/f1tenth_autonomous_racecar/assets/86895390/ff5455e6-bb97-4429-ba02-dd2966a43e03)

