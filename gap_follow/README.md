One of the simpler algorithms, gap follow takes lidar readings and essentially determines the largest "gap" in between potential obstacles to follow through. However, this can cause issues such as making a 180 and going back
in the wrong direction when making a sharp turn, as it may see the area the car came from as "the largest gap" as it is turning. This can be resolved by tuning multiple variables and making various optimizations, such as 
using a disparity extender. Below is the implementation in simulation 

https://github.com/Jixi123/f1tenth_autonomous_racecar/assets/86895390/f875de1d-0be9-40aa-9195-b79f9f64b5db

