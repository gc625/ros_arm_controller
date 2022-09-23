# Robot Arm Controller

This repo contains a stereo camera inverse kinematic control algorithm. The images from the cameras are processed using opencv. The joint positions are first calculated, and then using the following two algorithms, the forward kinematics are calculated.

# Forward Kinematic Algorithms

![alt text](docs/alg1.png)
![alt text](docs/alg1_2.png)

![alt text](docs/alg2.png)


# Joint detection results
For the following results, joint 2 was fixed while joints 1,3,4 are programmed to move in different sinusoidal patterns. Green denotes ground truth.


![alt text](docs/t2_j1.png)
![alt text](docs/t2_j3.png)
![alt text](docs/t2_j4.png)


# Error rates
![](docs/t2error.png)


