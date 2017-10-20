#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import math
import time
import types
import sys
import random

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

Goals = [(6, 3), (6,0), (4, -3), (-6, -3), (-6, 0), (-4, 3)]
stateCount = len(Goals)
Pubs = []
Robots = []
f = open("../obstacleLoc.txt", "r")
Obstacles = []
for x in f.readlines():
    obstaclePose = x.split()
    Obstacles.append((int(obstaclePose[0]), int(obstaclePose[1])))


def callback(data, args):
    global Robots
    [r, p, yaw] = euler_from_quaternion(
        [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
         data.pose.pose.orientation.w])
    #args[0] = current_Robot
    args[0].pose = [data.pose.pose.position.x, data.pose.pose.position.y, yaw]
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
    force_to_goal = [strength * math.cos(angle), strength * math.sin(angle)]
    if(mag <.4): #reached goal
        if(robot.state == 0):   #Need to stop to turn
            robot.turn = -1.5
        elif (robot.state == 3):  # Need to stop to turn
            robot.turn = 1.5
        if(robot.state == 1 or robot.state == 4):   #At load & unload
            robot.count = 10
            robot.wait = True
        if(robot.state == 2 or robot.state == 5):
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
    obstacleStrength = 1.3
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

    global Robots, Pubs

    count = 0
    for x in rospy.get_published_topics(namespace = '/'):
        if "robot" in x[0] and "base" in x[0]:
            rb = Robot()
            rb.count = 1+11*count #initial delay
            Robots.append(rb)
            rospy.Subscriber(x[0], Odometry, callback, callback_args=(Robots[count],count))
            count +=1
            topicName = x[0].split("/")
            Pubs.append(rospy.Publisher('/{}/cmd_vel'.format(topicName[1]), Twist, queue_size=10))
    rate = rospy.Rate(10)  # 10 Hz
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
            if(robot.turn != 0):
                if(abs(robot.pose[2]) > (abs(robot.turn)-0.2) and abs(robot.pose[2]) < (abs(robot.turn)+0.2)):
                    robot.turn = 0
                    robot.state = (robot.state + 1) % stateCount
                    break
                twist = Twist()
                twist.angular.z = -.5


            Pubs[count].publish(twist)
            count +=1

            rate.sleep()





if __name__ == '__main__':
    try:
        potential()
    except rospy.ROSInterruptException:
        pass