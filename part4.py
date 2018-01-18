# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot. 
# But this time, your speed will be about the same as the runaway bot. 
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random
    
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    
    ### Since our robot may run slower than the target robot, we use lazy hunter strategy:
    ### (1) we just watch and use EKF to estimate the next target position; hopefully by the end of 
    ###     this time, we get a good estimate of the next target position
    ### (2) we move to that position, which should be a point on the circle the target robot is on;
    ###     we know the target robot will be back later
    ### (3) catch the target robot when it is close
    
    WATCH_CYCLE = 20
    
    # initialize OTHER
    if not OTHER: # this is the first measurement
        OTHER = {}
        OTHER['chase_phase'] = 1
        OTHER['measurement'] = []
        OTHER['measurement_num'] = 0
    
    OTHER['measurement'].append(target_measurement)
    OTHER['measurement_num'] += 1
    
    # default output
    turning = 0.
    distance = 0.
    
    if 'x' not in OTHER and OTHER['measurement_num'] == 3:
        
        # calculate inital x and P
        x1, y1 = OTHER['measurement'][0]
        x2, y2 = OTHER['measurement'][1]
        x3, y3 = OTHER['measurement'][2]
        heading = atan2(y3-y2, x3-x2)
        heading_prev = atan2(y2-y1, x2-x1)
        # initial guess:
        x0 = x3
        y0 = y3
        theta0 = heading
        dist0 = (distance_between((x1,y1), (x2,y2))+distance_between((x2,y2), (x3,y3)))/2
        dtheta0 = angle_trunc(heading-heading_prev)
        
        # state matrix 
        x = matrix([[x0], [y0], [theta0], [dist0], [dtheta0]])
        
        # initial uncertainty:
        P = matrix([[1000., 0, 0, 0, 0],
                    [0, 1000, 0, 0, 0], 
                    [0, 0, 1000, 0, 0],
                    [0, 0, 0, 1000, 0],
                    [0, 0, 0, 0, 1000]])
        
        OTHER['x'] = x
        OTHER['P'] = P
        
    elif  'x' in OTHER:  # update x and P if they are in OTHER
        
        # retrieve state variables x and uncertianty P from OTHER:
        x = OTHER['x']
        P = OTHER['P']
        
        # time step
        dt = 1.
        
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
        Z = matrix([[target_measurement[0]], [target_measurement[1]]])
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
        
        # update OTHER
        OTHER['x'] = x
        OTHER['P'] = P
        
        target_next_pos = (x.value[0][0], x.value[1][0])
        
    if OTHER['chase_phase'] == 1 and OTHER['measurement_num'] == WATCH_CYCLE:
        OTHER['chase_phase'] = 2
        OTHER['next_pos'] = target_next_pos
    elif OTHER['chase_phase'] == 2:
        next_pos = OTHER['next_pos']
        heading_to_target = get_heading(hunter_position, next_pos)
        heading_difference = angle_trunc(heading_to_target - hunter_heading)
        turning =  heading_difference # turn towards the target
        distance = distance_between(hunter_position, next_pos)
        if distance >= max_distance:
            distance = max_distance # full speed ahead!
        elif distance == 0:
            OTHER['chase_phase'] = 3
    elif OTHER['chase_phase'] == 3:
        if distance_between(hunter_position, target_next_pos) < max_distance:
            next_pos = target_next_pos
            heading_to_target = get_heading(hunter_position, next_pos)
            heading_difference = angle_trunc(heading_to_target - hunter_heading)
            turning =  heading_difference # turn towards the target
            distance = distance_between(hunter_position, next_pos)
    return turning, distance, OTHER


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)
        
        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1            
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught



def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables
    
    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

print demo_grading(hunter, target, next_move)






