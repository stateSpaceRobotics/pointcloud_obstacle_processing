#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from NASA_ARMS.msg import *
from tf.transformations import euler_from_quaternion
import math
import time
import types
import sys
import random
import socket
from builtins import bytes

TCP_IP = '192.168.4.1'
TCP_PORT = 9999
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

class Robot:
    def __init__(self):
        self.wait = True
        #self.wait = False
        self.state = 4
        #self.state = 5
        self.pose = [0,0,0]
        self.waitingOn = None
        self.count = 0
        self.turn = 0

#Goals = [(3, -1), (5,0), (3, .5), (2, .5), (1, 0), (2, -.5)]
Goals = [(4, -.5), (5,0), (4, .3), (2, .3), (1, 0), (1, -.3)]
stateCount = len(Goals)
Pubs = []
Robots = []

#Obstacles = [(0, 0), (1, 3), (-1, 0.5)]
Obstacles = []

def obstacle_callback(obstacle_list):
    global Obstacles
    for obstacle in obstacle_list.points:
        Obstacles.append((obstacle.z, obstacle.x))  # offset to transform KinectV2 to base station
    
    rospy.logerr("Obstacles: {}".format(Obstacles[0]))

def callback(data, args):
    global Robots
    [r, p, yaw] = euler_from_quaternion(
        [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
         data.pose.pose.orientation.w])
    #args[0] = current_Robot
    #rospy.logerr("roll: {0:2f} | p: {1:2f} | yaw: {2:2f}".format(r, p, yaw))

    if(r<0):
        signR = -1
    else:
        signR = 1

    if(p<0):
        signP = -1
    else:
        signP = 1
    
    botAngle = (-1)*((90-90*signR)*signP + 60*p*signR)*(math.pi) / 180

    args[0].pose = [-1*data.pose.pose.position.z, data.pose.pose.position.x + .20, botAngle]
    
    #rospy.logerr("botAngle: {0:1f}".format(botAngle))

    #args[0].pose = [-1*data.pose.pose.position.z, data.pose.pose.position.x, yaw]
    rospy.logerr("pose is: {0:1f}, {1:1f}, {2:1f}".format(args[0].pose[0], args[0].pose[1], args[0].pose[2]))
    #args[1] = index of current_Robot
    Robots[args[1]] = args[0]

def goal_force(robot):
    #This should be used to scale the magnitude of the attractive goal force
    global Goals
    if(robot.count > 0):
        robot.count -= 1
        if(robot.count == 0):
            robot.wait = False
            robot.state = (robot.state + 1) % stateCount
        else:
            return [0,0]
    goal = Goals[robot.state]
    strength = .8
    force_to_goal = [0,0]
    #    1. Compute goal force vector and put it in the 'force_to_goal' variable
    diff_x = goal[0] - robot.pose[0]
    diff_y = goal[1] - robot.pose[1]
    mag = math.sqrt(diff_x**2+diff_y**2)
    direction = math.atan2(diff_y, diff_x)
    angle = wrap_angle(direction)
    #rospy.logerr("in goal_force, direction: {}, angle: {}".format(direction, angle))
    force_to_goal = [strength * math.cos(angle), strength * math.sin(angle)]
    rospy.logerr("Current goal: {}, goal_x: {}, goal_y: {}".format(robot.state, goal[0], goal[1]))
    if(mag <.5): #reached goal
	rospy.logerr("Goal reached: {}".format(robot.state))
        if(robot.state == 0):   #Need to stop to turn
	    robot.count = 10
            robot.state = (robot.state +1)% stateCount
	    
	    #robot.turn = -1.5
        elif (robot.state == 3):  # Need to stop to turn
            robot.count = 10
	    #robot.turn = 1.5
	    robot.state = (robot.state +1)% stateCount
        elif(robot.state == 1 or robot.state == 4):   #At load & unload
            robot.count = 10
            robot.wait = True
        elif(robot.state == 2 or robot.state == 5):
	    robot.count = 10
            robot.state = (robot.state +1)% stateCount

    # if (mag < .3 and not (robot.atLoad == True or robot.atUnload == True)):
    #     force_to_goal = [0,0]
    #     count = 25
    #     if (robot.goal == goals[0]):
    #         robot.atLoad = True
    #         robot.goal = goals[1]
    #     else:
    #         robot.atUnload = True
    #         robot.goal = goals[0]
    # print('goal force: ' + str(force_to_goal[0]) + ' ' + str(force_to_goal[1]) + ' mag: '+str(mag) + ' goal: '+str(robot.goal[0]))
    return force_to_goal

def obstacle_force(robot):
    distance = 10000
    global Goals
    global Obstacles
    global Robots
    obstacleStrength = .6
    vector = [0, 0]
    for r2 in Robots:
        distance = math.sqrt((robot.pose[0] - r2.pose[0]) ** 2 + (robot.pose[1] - r2.pose[1]) ** 2)
        strength = get_pf_magnitude_linear(distance)
        #strength = get_pf_magnitude_constant(distance)
        if(strength >0):
            if(robot.state == r2.state):
                goal = Goals[robot.state]
                r1ToGoal = math.sqrt((robot.pose[0] - goal[0]) ** 2 + (robot.pose[1] - goal[1]) ** 2)
                r2ToGoal = math.sqrt((r2.pose[0] - goal[0]) ** 2 + (r2.pose[1] - goal[1]) ** 2)
                if(r1ToGoal > r2ToGoal):
                    robot.wait = True #r2 is closer, let them go first
                    robot.waitingOn = r2
                    return [0,0] #rest doesn't matter because we are waiting
            if(robot.state == r2.state or r2.state == (robot.state +1) % stateCount): #Only not run into bots current or ahead of you
                vector[0] += (robot.pose[0] - r2.pose[0]) * strength
                vector[1] += (robot.pose[1] - r2.pose[1]) * strength
        else:
            if(robot.waitingOn == r2):
                robot.wait = False
                robot.waitingOn = False
        # vector[0] += (robot.pose[0] - r2.pose[0]) * strength
        # vector[1] += (robot.pose[1] - r2.pose[1]) * strength

    for obstacle in Obstacles:
        distance = math.sqrt((robot.pose[0] - obstacle[0]) ** 2 + (robot.pose[1] - obstacle[1]) ** 2)
        strength = get_pf_magnitude_linear(distance)
        #strength = get_pf_magnitude_constant(distance)
        vector[0] += (robot.pose[0] - obstacle[0]) * strength*obstacleStrength
        vector[1] += (robot.pose[1] - obstacle[1]) * strength*obstacleStrength
        #code below prevents running into obstacles directly ahead of bots
        if(vector[0] == 0 and vector[1] != 0):
            vector[0] = .3
        elif(vector[1] == 0 and vector[0] != 0):
            vector[1] = .3
    force_from_obstacles = vector
    return force_from_obstacles

def drive_from_force(force, robot):
    # This is multiplied by the angle of the drive force to get the turn command
    turn_multiplier = 1.0

    # If the absolute value of the angle of the force direction is greater than this, we only spin
    spin_threshold = math.pi / 2.5

    # This is multiplied by the magnitude of the force vector to get the drive forward command
    drive_multiplier = 2.0

    # The twist command to fill out and return
    twist = Twist()

    # Determine the angle and magnitude of the force
    force_angle = math.atan2(force[1], force[0])
    force_mag = math.hypot(force[0], force[1])

    angle_diff = wrap_angle(force_angle - robot.pose[2])
    #rospy.logerr("drive_from_force force_angle: {}, angle_diff: {}".format(force_angle, angle_diff))

    # Get turn speed
    twist.angular.z = turn_multiplier * angle_diff

    # Do we just spin
    if abs(angle_diff) < spin_threshold:
        twist.linear.x = drive_multiplier * force_mag

    return twist

def add_forces(a, b):
    # This function adds to force vectors together and returns the result
    assert len(a) == len(b), "Force vectors differ in length"
    c = [a[i] + b[i] for i in range(len(a))]
    return c

def wrap_angle(angle):
    # This function will take any angle and wrap it into the range [-pi, pi]
    while angle >= math.pi:
        angle = angle - 2 * math.pi

    while angle <= -math.pi:
        angle = angle + 2 * math.pi
    return angle

def get_pf_magnitude_linear(distance):
    # How close to the obstacle do we have to be to begin feeling repulsive force
    distance_threshold = 2.0
    # The maximum strength of the repulsive force
    max_strength = 1
    if distance < distance_threshold:
        return ((distance_threshold - distance) / distance_threshold) * max_strength

    return 0

def get_pf_magnitude_constant(distance):
    # How close to the obstacle do we have to be to begin feeling repulsive force
    distance_threshold = 1.0
    # Strength of the repulsive force
    strength = 1.0
    if distance < distance_threshold:
        return strength
    return 0



def potential():
    rospy.init_node('MiniBots', anonymous=True)  # Initialize the ros node
    sub_once = rospy.Subscriber('/obstacles', PointIndicesArray, obstacle_callback)
    rospy.logerr("------------------------------------------------------waiting---------------------------------")
    rospy.wait_for_message('/obstacles', PointIndicesArray)
    sub_once.unregister()
    global Robots, Pubs

    count = 0
    #for x in rospy.get_published_topics(namespace = '/'):
    #    if "robot" in x[0] and "base" in x[0]:
    #        rb = Robot()
    #        rb.count = 1+11*count #initial delay
    #        Robots.append(rb)
    #        rospy.Subscriber(x[0], Odometry, callback, callback_args=(Robots[count],count))
    #        count +=1
    #        topicName = x[0].split("/")
    #        Pubs.append(rospy.Publisher('/{}/cmd_vel'.format(topicName[1]), Twist, queue_size=10))
    rate = rospy.Rate(5)  # 10 Hz
    rb = Robot()
    Robots.append(rb)
    rb.count = 1
    rospy.Subscriber('/device01', Odometry, callback, callback_args=(rb,0))
    Pubs.append(rospy.Publisher('/cmd_vel'.format(), Twist, queue_size=10))
    #Create our publisher to send drive commands to the robot

    while not rospy.is_shutdown():

        #pub.publish(twist)
        count = 0
        for robot in Robots:
            # 2. Compute obstacle avoidance force
            o_force = obstacle_force(robot)

            # 1. Compute attractive force to goal
            g_force = goal_force(robot)

            # 3. Get total force by adding together
            total_force = add_forces(g_force, o_force)

            # 4. Get final drive command from total force
            twist = drive_from_force(total_force, robot)

            # 5. Publish drive command, then sleep
            if(robot.wait):
                twist = Twist()
            motorForce(total_force, robot)
            '''
	    if(robot.turn != 0):
                if(abs(robot.pose[2]) > (abs(robot.turn)-0.2) and abs(robot.pose[2]) < (abs(robot.turn)+0.2)):
                    robot.turn = 0
                    robot.state = (robot.state + 1) % stateCount
                    break
                twist = Twist()
                twist.angular.z = -.5
	   

            Pubs[count].publish(twist)
            count +=1
            '''
            rate.sleep()


def motorForce(force, robot):
    force_angle = math.atan2(force[1], force[0])
    angle_diff = wrap_angle(force_angle - robot.pose[2])
    #limits to angle dif to prevent negative values
    
    if(angle_diff > 3):
        angle_diff = 3
    elif(angle_diff < -3):
        angle_diff = -3
    
    force_mag = math.hypot(force[0], force[1])
    midSpeed = 100
    minSpeed = 20
    maxSpeed = 100
    scalar = .5
    baseSpeed = 95
    combinedSpeed = midSpeed*force_mag
    speedDif = combinedSpeed*angle_diff*scalar
    left = baseSpeed+speedDif
    right = baseSpeed-speedDif
    
    rospy.logerr("motorForce force_angle: {}, botAngle: {}, angle_diff: {}, speedDif: {}, left: {}, right: {}".format(force_angle, robot.pose[2], angle_diff, speedDif, left, right))

    if(left > maxSpeed):
        left = maxSpeed
    elif(left < minSpeed):
        left = minSpeed
    if (right > maxSpeed):
        right = maxSpeed
    elif (right < minSpeed):
        right = minSpeed
    motorCommand = "{0:.0f},{1:.0f}".format(left, right)
    
    left = int(left)
    right = int(right)

    if(robot.wait):
        left = 70
        right = 70
    command = bytes([left, right])
    #rospy.logerr('---------L: {0}, R: {1}, yawErr: {2}, Mag {3}, a_d: {4}'.format(left,right, speedDif, force_mag, angle_diff))
    s.send(command)
    #if (robot == Robots[1]):
     #   print(motorCommand)
        #bob = 2+1

    #print(motorCommand)
    return motorCommand

    


if __name__ == '__main__':
    try:
        potential()
    except rospy.ROSInterruptException:
        pass
