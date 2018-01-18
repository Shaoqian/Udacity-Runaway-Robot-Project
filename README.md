# Udacity-Runaway-Robot-Project
This is my code for the runaway robot project for the course 'Artificial Intelligence for Robotics' on Udacity

Background:

A robotics company named Trax has created a line of small self-driving robots designed to autonomously traverse desert environments in search of undiscovered water deposits.

A Traxbot looks like a small tank. Each one is about half a meter long and drives on two continuous metal tracks. In order to maneuver itself, a Traxbot can do one of two things: it can drive in a straight line or it can turn. So to make a  right turn, A Traxbot will drive forward, stop, turn 90 degrees, then continue driving straight.

This series of questions involves the recovery of a rogue Traxbot. This bot has gotten lost somewhere in the desert and is now stuck driving in an almost-circle: it has been repeatedly driving forward by some step size, stopping, turning a certain amount, and repeating this process... Luckily, the Traxbot is still sending all of its sensor data back to headquarters.

Idea:

(1) In part 1, there is no measurement noise for the location of the target robot. Therefore, three measurements are enough to determine its next location. However, I have still implemented a moving average filter, which serves as an working example for part 2. 

(2) In part 2, measurements of the target robot are noisy. I have used Extended Kalman Filter (EKF) to estimate the robot's next position. The states of the robots are [x, y, heading, distance, turning]. Since the system equations are nonlinear, EKF is necessary. I tried to use particle filters to do the esitimation. Intuitively, every particle should include all the information of the robot, i.e., its five states, therefore, I think particle filter is not a good choice here. 

(3) In part 4, since our robot moves slower than the target robot, one can look a number of steps ahead and try to catch the target by moving straight instead of moving on a circle. Here I have used to a lazy approach, waiting on the path of the target. 

(4) I didn't solve part 5. My EKF doesn't converge, or distance converges to a negative value, because of the large noise. I have tried another approach which doesn't work either. My approach was as follows. 

Since it is easy to show that the discrete positions of the target robot are on a circle, one can model the motion of the target as:
 
xk = cx + r * cos(omega * k + theta) + wk,
yk = cy + r * sin(omega * k + theta) + vk, 

where (cx, cy) is the center of the circle, r is the radius, omega is the angular velocity of the target, and theta is its initial phase angle. 

One can estimate (cx, cy, r, omega, theta) by minimizing f = sum ((cx+r * cos(omega * k+theta)-xk)^2 + (cy+r*sin(omega * k+theta)-yk)^2). Note that f is not convex. A good initial guess can be achieved by doing DFT of x and y, since x, y are corrupted sinusoids. Once initial guess is acquired, gradient based search can be used. After one gets the function of the circle, our robot can move on the circle and wait for the target. 

Acknowledgement:

The posts in the Udacity forum https://discussions.udacity.com/t/kalman-filter-for-runaway-robot/37966, and especially Jeremy Shannon's work https://github.com/jeremy-shannon/udacity-AI-for-robotics/blob/master/Project%20-%20Runaway%20Robot/studentMain.py have helped me a lot in doing this project. 


