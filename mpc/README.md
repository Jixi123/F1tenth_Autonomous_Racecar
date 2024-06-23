# Model Predictive Control (MPC)

MPC is a control method which allows for optimal control. We essentially provide the algorithm an optimal reference trajectory, and at each time step a sequence of controls is calculated by MPC that would get as close to the optimal trajectory as possible. However, only the first command in the predicted sequence is executed, and and the predicted sequence is updated at each timestep (multiple times per second). This allows for much smoother driving as opposed to RRT and Pure Pursuit, which both tend to swerve back and forth due to the nature of their algorithms.  
![IMG_3173](https://github.com/Jixi123/F1tenth_Autonomous_Racecar/assets/86895390/b0083b4a-d97d-4151-a5d0-5824f96d6f0a)
