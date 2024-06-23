# Rapidly-Exploring Random Tree (RRT)

RRT is a map based method that is one step above Pure Pursuit in the sense that it also allows for dynamic obstacle avoidance. To do this, at each timestep, a new RRT tree is created in which nodes will stop being added once one gets close enough to a target goal, which in this case will be the closest preprocessed waypoint. Since nodes can't be added in spaces where an obstacle is situated, a path that is obstacle-free will thus be generated. Since this path is updated multiple times per second, it can avoid obstacles that are placed in front of the car as the car is moving. 

In this clip, we can see the car swerve to avoid the foot and the trash can. 
https://github.com/Jixi123/F1tenth_Autonomous_Racecar/assets/86895390/ff354f01-9fad-472c-b2cb-f26267a40119

