#Wall Follow

The Wall follow algorithm calculates the distance the car is from either the left or right wall, and uses this information to steer such that a specificed distance is maintained between the car and the wall. 
Being one of the simplest algorithms, Wall following by nature won't work on tracks with 90 degree or similarly sharp turns. Below is the implementation in simulation. In this implementation, it attempts to track the left wall, 
and you can see at the bottom, due to the nature of wall following, nearly gets stuck in the opening on the left, since the car want's to get closer to the left wall. 

https://github.com/Jixi123/f1tenth_autonomous_racecar/assets/86895390/b2fa2a3d-0ddd-46a5-acd2-79627739a030

