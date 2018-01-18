# ----------
# Part Five
#
# This time, the sensor measurements from the runaway Traxbot will be VERY 
# noisy (about twice the target's stepsize). You will use this noisy stream
# of measurements to localize and catch the target.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3 and 4. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random
from numpy import *

def parameter_estimate(fftx, ffty, N):
    # This function estimates cx, cy, r, w, theta from N point fftw
    
    # estimate cx, r, w, theta from x measurements
    fftx_temp = abs(fftx)
    fftx_temp[0] = 0.
    index1 = max(xrange(len(fftx_temp)), key=fftx_temp.__getitem__)
    fftx_temp[index1] = 0.
    index2 = max(xrange(len(fftx_temp)), key=fftx_temp.__getitem__)
    if index1 > index2:
        temp = index2
        index2 = index1
        index1 = temp
    val1 = abs(fftx)[index1]    
    val2 = abs(fftx)[index2]
    
    cx = real(fftx[0])  # cw 
    rx = val1 + val2   # r
    wx = (index1+N-index2)*pi/N  # w
    thetax = (angle(fftx[index1])-angle(fftx[index2]))/2.0  # theta
    
    # estimate cy, r, w, theta from y measurements
    ffty_temp = abs(ffty)
    ffty_temp[0] = 0.
    index1 = max(xrange(len(ffty_temp)), key=ffty_temp.__getitem__)
    ffty_temp[index1] = 0.
    index2 = max(xrange(len(ffty_temp)), key=ffty_temp.__getitem__)
    if index1 > index2:
        temp = index2
        index2 = index1
        index1 = temp
    val1 = abs(ffty)[index1]    
    val2 = abs(ffty)[index2]
    
    
    cy = real(ffty[0])  # cw 
    ry = val1 + val2   # r
    wy = (index1+N-index2)*pi/N  # w
    thetay = (angle(ffty[index1])-angle(ffty[index2]))/2.0 +pi/2  # theta
    
    # average r, w, and theta
    r = (rx+ry)/2.0
    w = (wx+wy)/2.0
    theta = (thetax+thetay)/2.0
    print 'inital guess cx, cy, r, w, theta = ', cx, cy, r, w, theta
    return cx, cy, r, w, theta

def gradient_search(x, y, N, cx, cy, r, w, theta, stepsize, iteration_num):
   
    for i in range(iteration_num):
    	gradcx = 0.
        gradcy = 0. 
        gradr = 0. 
        gradw = 0.
        gradtheta = 0. 
    
        for k in range(N):
            xk = x[k]
            yk = y[k]
            S = sin(w*k+theta)
            C = cos(w*k+theta)
            P = cx + r*C - xk
            Q = cy + r*S - yk
        
            gradcx += P
            gradcy += Q
            gradr += P*C+Q*S
            gradw += P*(-r)*k*S + Q*r*k*C
            gradtheta += P*(-r)*S + Q*r*C

	    # scale gradient    
        gradcx /= N**2
        gradcy /= N**2
        gradr /= N**2
        gradw /= N**2
       	gradtheta /= N**2

        print 'gradient = ', gradcx, gradcy, gradr, gradw, gradtheta
    
        cx += (-gradcx)*stepsize
        cy += (-gradcy)*stepsize
        r += (-gradr)*stepsize
        w += (-gradw)*stepsize
        theta += (-gradtheta)*stepsize
	
        print 'updated parameter cx, cy, r, w, theta = ', cx, cy, r, w, theta

    return cx, cy, r, w, theta




def hunter_move_towards(hunter_position, hunter_heading, next_position, max_distance):
    # This function generate turning and distance for hunter's next move
    
    heading_to_target = get_heading(hunter_position, next_position)
    turning  = angle_trunc(heading_to_target - hunter_heading)
    distance = distance_between(hunter_position, next_position)
    if distance > max_distance:
        distance = max_distance
    return turning, distance
    
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    
    ### Lazy hunter approach
    ### idea: (1) chase_phase 1: we just watch and collect data; at the end of the phase, we estimate 
    ### cx, cy, r, w, theta; (2) chase_phase 2: move hunter robot to point (cx, cy-r); 
    ### (3) update the parameters and adjust the position of the hunter robot; if the target robot is 
    ### within one step away, catch it. 
    
    ### the target robot motion is modelled as:
    ### x = cx + r * cos(w*k + theta); y = cy + r * sin(w*k +theta)

    WATCH_CYCLE = 500
    
    if not OTHER: # first time calling this function, set up my OTHER variables.
        OTHER = {}
        OTHER['chase_phase'] = 1
        OTHER['x'] = []
        OTHER['y'] = []
        OTHER['measurement_num'] = 0
    	OTHER['cx'] = 0.
    	OTHER['cy'] = 0.
    	OTHER['r'] = 0.
    	OTHER['w'] = 0.
    	OTHER['theta'] = 0.
		
    OTHER['x'].append(target_measurement[0])     # save data, initiate turning and distance
    OTHER['y'].append(target_measurement[1])
    OTHER['measurement_num'] += 1
    turning = 0.
    distance = 0.
    
    if OTHER['chase_phase'] == 1:
        if OTHER['measurement_num'] >= WATCH_CYCLE: 
            OTHER['chase_phase'] = 2
    elif OTHER['chase_phase'] == 2 and 'next_pos' not in OTHER:
        # since x, y measurement are corrupted sinusoids, we use fft to estimate constant terms, frequency, 
        # amplitude and initial phase angle; when multiple estimates are available, we simply average them

        N = OTHER['measurement_num']
        fftx = fft.fft(OTHER['x'])/N
        ffty = fft.fft(OTHER['y'])/N
        
        cx, cy, r, w, theta = parameter_estimate(fftx, ffty, N)
        x_next = cx
        y_next = cy - r
        next_pos = (x_next, y_next)

    	# update OTHER
    	OTHER['cx'] = cx
    	OTHER['cy'] = cy
    	OTHER['r'] = r
    	OTHER['w'] = w
    	OTHER['theta'] = theta
    	OTHER['next_pos'] = next_pos

        turning, distance = hunter_move_towards(hunter_position, hunter_heading, next_pos, max_distance)
    	print 'initial lowest point on circle = ', next_pos
        

    elif OTHER['chase_phase'] == 2 and 'next_pos' in OTHER:
        next_pos = OTHER['next_pos']
	
        turning, distance = hunter_move_towards(hunter_position, hunter_heading, next_pos, max_distance)
        if hunter_position == next_pos:
            OTHER['chase_phase'] = 3

    elif OTHER['chase_phase'] == 3:
    	# for every new measurement, use gradient based search update our parameters for iteration_num steps
        # calculate target robot next postion, and the new (cx, cy-r)
        # if distance_between(hunter_position, next_pos) > max_distance, hunter go to new (cx, cy-r), 
        # else catch the target robot

	    # retrieve information
        x = OTHER['x']
    	y = OTHER['y']
    	N = OTHER['measurement_num']
    	cx = OTHER['cx'] 
    	cy = OTHER['cy']  
    	r = OTHER['r']  
    	w = OTHER['w']  
    	theta = OTHER['theta'] 
    	 
    	stepsize = 0.02*max_distance
    	iteration_num = 1
    	
    	# gradient based search
    	cx, cy, r, w, theta = gradient_search(x, y, N, cx, cy, r, w, theta, stepsize, iteration_num)
    
    	# update OTHER
    	OTHER['cx'] = cx
    	OTHER['cy'] = cy
    	OTHER['r'] = r
    	OTHER['w'] = w
    	OTHER['theta'] = theta
	

        x_next = cx
        y_next = cy - r
        next_pos = (x_next, y_next)
        target_next_pos = (cx + r * cos(w*N + theta),  cy + r * sin(w*N+theta))
        print 'update next position ', next_pos

        if distance_between(target_next_pos, hunter_position) > max_distance:
            turning, distance = hunter_move_towards(hunter_position, hunter_heading, next_pos, max_distance)
        else:
            turning, distance = hunter_move_towards(hunter_position, hunter_heading, target_next_pos, max_distance)

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
    max_distance = 0.97 * target_bot.distance # 0.97 is an example. It will change.
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
measurement_noise = 2*target.distance # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

print demo_grading(hunter, target, next_move)






