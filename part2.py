# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess 
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered 
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position. 
#
# ----------
# GRADING
# 
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random

# This is the function you have to write. Note that measurement is a 
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be 
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
    
    # comment: estimating next postion using EKF, initial condition is very
    # important. 
    if not OTHER: # this is the first measurement
        OTHER = [1, measurement]
        return (0, 0), OTHER    # dummy next position
    elif OTHER[0] == 1:
        OTHER = [2, OTHER[1], measurement]
        return (0, 0), OTHER
    elif OTHER[0] == 2:
        x1, y1 = OTHER[1]
        x2, y2 = OTHER[2]
        x3, y3 = measurement
        heading = atan2(y3-y2, x3-x2)
        heading_prev = atan2(y2-y1, x2-x1)
        # initial guess: EKF state: [x, y, heading, distance, turning]
        x0 = x3
        y0 = y3
        theta0 = heading
        dist0 = (distance_between((x1,y1), (x2,y2))+distance_between((x2,y2), (x3,y3)))/2
        dtheta0 = angle_trunc(heading-heading_prev)
        # initial uncertainty:
        P = matrix([[1000., 0, 0, 0, 0],
                    [0, 1000, 0, 0, 0], 
                    [0, 0, 1000, 0, 0],
                    [0, 0, 0, 1000, 0],
                    [0, 0, 0, 0, 1000]])
        # rebuild OTHER [x,P]
        OTHER = [[],[]]
                    
    else:
        # retrieve state variables x and uncertianty P from OTHER:
        x0 = OTHER[0].value[0][0]
        y0 = OTHER[0].value[1][0]
        theta0 = OTHER[0].value[2][0] % (2*pi)
        dist0 = OTHER[0].value[3][0]
        dtheta0 = OTHER[0].value[4][0]
        P = OTHER[1]
    
    # time step 
    dt = 1.
    
    # state matrix 
    x = matrix([[x0], [y0], [theta0], [dist0], [dtheta0]])
    
    # external motion
    u = matrix([[0.], [0.], [0.], [0.], [0.]])
    
    # measurement function:
    H = matrix([[1., 0, 0, 0, 0],
                [0, 1, 0, 0, 0]])
    
    # measurement uncertainty:
    R = matrix([[measurement_noise, 0], 
                [0, measurement_noise]])
    
    # identity matrix:
    I = matrix([[]])
    I.identity(5)
    
    # measurement update
    Z = matrix([[measurement[0]], [measurement[1]]])
    y = Z - H*x
    S = H*P*H.transpose() + R
    K = P*H.transpose()*S.inverse()
    x = x+(K*y)
    P = (I-(K*H))*P
    
    # retrieve current estimates based on measurement update
    x0 = x.value[0][0]
    y0 = x.value[1][0]
    theta0 = x.value[2][0]
    dist0 = x.value[3][0]
    dtheta0 = x.value[4][0]
    
    # prediction:
    x = matrix([[x0+dist0*cos(theta0+dtheta0)],   # next state
                [y0+dist0*sin(theta0+dtheta0)], 
                [theta0+dtheta0],
                [dist0], 
                [dtheta0]])
                
    F = matrix([[1, 0, -dist0*sin(theta0+dtheta0), cos(theta0+dtheta0), -dist0*sin(theta0+dtheta0)], # Jacobian
                [0, 1,  dist0*cos(theta0+dtheta0), sin(theta0+dtheta0),  dist0*cos(theta0+dtheta0)], 
                [0, 0, 1, 0, dt],
                [0, 0, 0, 1, 0], 
                [0, 0, 0, 0, 1]])

    P = F*P*F.transpose()
    
    OTHER[0] = x
    OTHER[1] = P
    
    xy_estimate = (x.value[0][0], x.value[1][0])
    return xy_estimate, OTHER

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any 
# information that you want. 
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that 
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER 
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos, test_target)





