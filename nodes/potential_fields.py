#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import math
import time
from datetime import datetime as dt
from threading import Thread
import multiprocessing as mp

#####################
# BEGIN Global Variable Definitions
class Robot:
    def __init__(self, goal):
        self.goal = goal
        self.wait = False
        self.atLoad = False
        self.atUnload = False
        self.pose = [0,0,0]


count = 0
count2 = 0
laser_scan = None
goal = [7,0,0]
goalBeginning = [-7,0,0]
goals = [goal, goalBeginning]
goalCount = 1
robot = Robot(goal)
robot2 = Robot(goal)
# END Global Variable Definitions
#####################+

#####################
# BEGIN ROS Topic Callback Functions [DON'T MESS WITH THIS STUFF]
#####################


def robot_callback(data):
    #This function updates the robots position and yaw, based on the ground truth (we don't have localization yet)
    global robot
    [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    robot.pose = [data.pose.pose.position.x,data.pose.pose.position.y,yaw]

def robot2_callback(data):
    #This function updates the robots position and yaw, based on the ground truth (we don't have localization yet)
    global robot2
    [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    robot2.pose = [data.pose.pose.position.x,data.pose.pose.position.y,yaw]

def laser_callback(data):
    #This function sets the global laser_scan variable to hold the most recent laser scan data
    global laser_scan
    laser_scan = data

def goalCallback(data):
    #This function will update the goal of the robot
    global goal
    goal = [data.x, data.y]

#####################
##### END CALLBACK FUNCTIONS   
#####################

##################### 
# BEGIN HELPER FUNCTIONS [USE THESE IN YOUR CODE, BUT YOU SHOULDN'T NEED TO MODIFY]
#####################

def add_forces(a, b):
    #This function adds to force vectors together and returns the result
    assert len(a) == len(b), "Force vectors differ in length"
    c = [a[i] + b[i] for i in range(len(a))]
    return c

def wrap_angle(angle):
    #This function will take any angle and wrap it into the range [-pi, pi]
    while angle >= math.pi:
        angle = angle - 2*math.pi
        
    while angle <= -math.pi:
        angle = angle + 2*math.pi
    return angle

#####################
##### END HELPER FUNCTIONS
#####################

#####################
# BEGIN MODIFIABLE LAB CODE [ALTHOUGH MOST MODIFICATIONS SHOULD BE WHERE SPECIFIED]
#####################

#This function takes in a force [x,y] (in robot coordinates) and returns the drive command (Twist) that should be sent to the robot motors
def drive_from_force(force, robot):
    
    #####################################################
    #PARAMETERS : MODIFY TO GET ROBOT TO MOVE EFFECTIVELY
    
    #This is multiplied by the angle of the drive force to get the turn command 
    turn_multiplier = 1.0
    
    #If the absolute value of the angle of the force direction is greater than this, we only spin
    spin_threshold = math.pi / 2.5
    
    #This is multiplied by the magnitude of the force vector to get the drive forward command
    drive_multiplier = 2.0
    
    #END OF PARAMETERS
    #####################################################

    #The twist command to fill out and return
    twist = Twist()

    #Determine the angle and magnitude of the force
    force_angle = math.atan2(force[1],force[0])
    force_mag = math.hypot(force[0],force[1])

    angle_diff = wrap_angle(force_angle - robot.pose[2])

    #Get turn speed
    twist.angular.z = turn_multiplier * angle_diff

    #Do we just spin
    if abs(angle_diff) < spin_threshold:
        twist.linear.x = drive_multiplier * force_mag

    return twist

# This function determines and returns the attractive force (force_to_goal) to the goal.  
# This force should be in robot coordinates
def goal_force():
    global goals
    global count
    global robot #format [x_position, y_position, yaw]

    if (count != 0):
        count -=1
        return [0,0]
    else:
        robot.atUnload = False
        robot.atLoad = False
    #This should be used to scale the magnitude of the attractive goal force
    strength = .9
    force_to_goal = [0,0]
    #    1. Compute goal force vector and put it in the 'force_to_goal' variable
    diff_x = robot.goal[0] - robot.pose[0]
    diff_y = robot.goal[1] - robot.pose[1]
    mag = math.sqrt(diff_x**2+diff_y**2)
    direction = math.atan2(diff_y, diff_x)
    angle = wrap_angle(direction)
    force_to_goal = [strength * math.cos(angle), strength * math.sin(angle)]

    if (mag < .3 and not (robot.atLoad == True or robot.atUnload == True)):
        force_to_goal = [0,0]
        count = 25
        if (robot.goal == goals[0]):
            robot.atLoad = True
            robot.goal = goals[1]
        else:
            robot.atUnload = True
            robot.goal = goals[0]
    print('goal force: ' + str(force_to_goal[0]) + ' ' + str(force_to_goal[1]) + ' mag: '+str(mag) + ' goal: '+str(robot.goal[0]))
    return force_to_goal


def goal_force2():
    # This is the robot's actual global location, set in robot_callback
    global goals
    global count2
    global robot2  # format [x_position, y_position, yaw]

    if (count2 != 0):
        count2 -= 1
        return [0, 0]
    else:
        robot2.atUnload = False
        robot2.atLoad = False
    # This should be used to scale the magnitude of the attractive goal force
    strength = .9
    force_to_goal = [0, 0]
    #    1. Compute goal force vector and put it in the 'force_to_goal' variable
    diff_x = robot2.goal[0] - robot2.pose[0]
    diff_y = robot2.goal[1] - robot2.pose[1]
    mag = math.sqrt(diff_x ** 2 + diff_y ** 2)
    direction = math.atan2(diff_y, diff_x)
    angle = wrap_angle(direction)
    force_to_goal = [strength * math.cos(angle), strength * math.sin(angle)]

    if (mag < .3 and not (robot2.atLoad == True or robot2.atUnload == True)):
        force_to_goal = [0, 0]
        count2 = 25
        if (robot2.goal == goals[0]):
            robot2.atLoad = True
            robot2.goal = goals[1]
        else:
            robot2.atUnload = True
            robot2.goal = goals[0]
    print('goal force2: ' + str(force_to_goal[0]) + ' ' + str(force_to_goal[1]) + ' mag: ' + str(mag) + ' goal: '+str(robot2.goal[0]))
    return force_to_goal

# def waitThread():
#     global goals
#     global robot
#
#     if (robot.goal == goals[0]):
#         robot.atLoad = True
#         robot.atLoad = False
#         robot.goal = goals[1]
#         print("goal: "+str(robot.goal[0]))
#
#     else:
#         robot.atUnload = True
#         time.sleep(1)
#         robot.atUnload = False
#         robot.goal = goals[0]
#         print(str(robot.goal[0]))
#         # robot.atUnload = False
#
# def waitThread2():
#     global goals
#     global robot2
#
#     if (robot2.goal == goals[0]):
#         robot2.atLoad = True
#         time.sleep(1)
#         robot2.atLoad = False
#         robot2.goal = goals[1]
#         print(str(robot2.goal[0]))
#
#     else:
#         robot2.atUnload = True
#         time.sleep(1)
#         robot2.atUnload = False
#         robot2.goal = goals[0]
#         print(str(robot.goal[0]))
#         # robot2.atUnload = False

def waitForDirtToBeDeposited(robot):
    global goals
    global count
    if(robot.goal == goals[0]):
        robot.atLoad = True
        print(count)
        #thread = Thread(target=waitThread(robot))
        #thread.start();
        # waitSomeTime Idle

    else:
        robot.atUnload = True
        robot.goal = goals[0]

        #robot.atUnload = False


def obstacle_force_logan():
    global robot
    global robot2
    distance = 10000
    obstacle1 = [0,0.3,0]
    obstacle2 = [-2, -0.4, 0]
    obstacle3 = [robot2.pose[0], robot2.pose[1], 0]
    obstacles = [obstacle1, obstacle2, obstacle3]
    vector = [0,0]
    for obstacle in obstacles:
        distanceTemp = math.sqrt((robot.pose[0] - obstacle[0]) ** 2 + (robot.pose[1] - obstacle[1]) ** 2)
        distance = distanceTemp
        strength = get_pf_magnitude_constant(distance)
        if(obstacle == obstacle3 and strength > 0):
            if(robot2.atLoad or robot2.atUnload):
                robot.wait = True
            else:
                robot.wait = False
        vector[0] += (robot.pose[0] - obstacle[0]) * strength
        vector[1] += (robot.pose[1] - obstacle[1]) * strength
    force_from_obstacles = vector
    return force_from_obstacles

def obstacle_force_logan2():
    global robot
    global robot2
    distance = 10000
    obstacle1 = [0,0.3,0]
    obstacle2 = [-2, -0.4, 0]
    obstacle3 = [robot.pose[0], robot.pose[1], 0]
    obstacles = [obstacle1, obstacle2, obstacle3]
    vector = [0,0]
    for obstacle in obstacles:
        distanceTemp = math.sqrt((robot2.pose[0] - obstacle[0]) ** 2 + (robot2.pose[1] - obstacle[1]) ** 2)
        distance = distanceTemp
        strength = get_pf_magnitude_constant(distance)
        if (obstacle == obstacle3 and strength > 0):
            if (robot.atLoad or robot.atUnload):
                robot2.wait = True
            else:
                robot2.wait = False
        vector[0] += (robot2.pose[0] - obstacle[0])*strength
        vector[1] += (robot2.pose[1] - obstacle[1])*strength
    force_from_obstacles = vector
    return force_from_obstacles
#This function looks at the current laser reading, then computes and returns the obstacle avoidance force vector (in local robot coordinates)
def obstacle_force():

    #The most recent laser_scan.  It has the following fields 
    #   laser_scan.angle_min : angle of the first distance reading
    #   laser_scan.angle_increment : the angular difference between consecutive distance readings
    #   laser_scan.ranges : an array all of the distance readings
    global laser_scan
    
    #Only run if we have a laser scan to work with
    if laser_scan is None:
        return [0,0]

    #The obstacle repulsion force variable, will be returned
    #This will accumulate all of the obstacle forces acting upon us
    force_from_obstacles = [0,0]

    cur_angle = laser_scan.angle_min 
    #cur_angle will always have the relative angle between the robot's yaw and the current laser reading

    for i in range(len(laser_scan.ranges)):

        # Get the magnitude of the repulsive force for this distance reading
        # CHANGE WHICH FUNCTION IS CALLED FOR LAB 2 PART C
        strength = get_pf_magnitude_constant(laser_scan.ranges[i])
    
        #########################
        # LAB 2 PART B : BEGIN
        #########################

        # PART B CODE HERE: 
        #    1. Compute force vector with magnitude 'strength' away from obstacle
        #    2. Add this force vector to the 'force_from_obstacles' vector
    
        x_pos = math.cos(wrap_angle(robot.pose[2] + cur_angle + math.pi)) * strength * laser_scan.angle_increment
        y_pos = math.sin(wrap_angle(robot.pose[2] + cur_angle + math.pi)) * strength * laser_scan.angle_increment
        cur_obstacle_force =[x_pos, y_pos] 
        force_from_obstacles = add_forces(force_from_obstacles, cur_obstacle_force)

        #########################
        # LAB 2 PART B : END
        #########################

        cur_angle = cur_angle + laser_scan.angle_increment

    print('obstacle goal: ' + str(force_from_obstacles[0]) + ' ' + str(force_from_obstacles[1]))
    return force_from_obstacles

# This function returns the magnitude of repulsive force for the input distance
# using a linear drop-off function
def get_pf_magnitude_linear(distance):

    #####################################################
    #PARAMETERS: MODIFY TO GET THINGS WORKING EFFECTIVELY
        
    #How close to the obstacle do we have to be to begin feeling repulsive force
    distance_threshold = 1.0

    #The maximum strength of the repulsive force
    max_strength = 1.3

    #END OF PARAMETERS
    #####################################################
    
    #########################
    # LAB 2 PART C : BEGIN
    #########################

    # PART C CODE HERE: 
    #   1. Compute the magnitude of the force for the given distance and return it
    if distance < distance_threshold:
        return ((distance_threshold - distance) / distance_threshold) * max_strength
        # print(strength)
        # return strength
    #########################
    # LAB 2 PART C : END
    #########################

    return 0

# This function returns the magnitude of repulsive force for the input distance
# using a constant value if the obstacles is closer than a threshold
def get_pf_magnitude_constant(distance):

    #####################################################
    #PARAMETERS: MODIFY TO GET THINGS WORKING EFFECTIVELY
        
    #How close to the obstacle do we have to be to begin feeling repulsive force
    distance_threshold = 1.0 

    #Strength of the repulsive force
    strength = 1.0

    #END OF PARAMETERS
    #####################################################

    if distance < distance_threshold:
        return strength

    return 0

# This is the main loop of the lab code.  It runs continuously, navigating our robot
# (hopefully) towards the goal, without hitting any obstacles
def potential():
    rospy.init_node('lab2', anonymous=True) #Initialize the ros node
    pub = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=10) #Create our publisher to send drive commands to the robot
    pub2 = rospy.Publisher('robot_1/cmd_vel', Twist, queue_size=10)
    #rospy.Subscriber("robot_1/base_scan", LaserScan, laser_callback) #Subscribe to the laser scan topic
    rospy.Subscriber("robot_0/base_pose_ground_truth", Odometry, robot_callback) #Subscribe to the robot pose topic
    rospy.Subscriber("robot_1/base_pose_ground_truth", Odometry, robot2_callback) #Subscribe to the robot pose topic
    rospy.Subscriber("next_waypoint", Point, goalCallback)#Subscribe to the goal location topic

    rate = rospy.Rate(10) #10 Hz    
    
    while not rospy.is_shutdown():
        
        #Don't do anything until the goal location has been received
        # if goal is None:
        #     rate.sleep()
        #     continue

        #1. Compute attractive force to goal
        g_force = goal_force()
        g_force2 = goal_force2()

        #2. Compute obstacle avoidance force
        o_force = obstacle_force_logan()
        o_force2 = obstacle_force_logan2()

        #3. Get total force by adding together
        total_force = add_forces(g_force, o_force)
        total_force2 = add_forces(g_force2, o_force2)

        #4. Get final drive command from total force
        twist = drive_from_force(total_force, robot)
        twist2 = drive_from_force(total_force2, robot2)

        #5. Publish drive command, then sleep
        if(robot.atLoad or robot.atUnload or robot.wait):
            twist = Twist()
        if(robot2.atLoad or robot2.atUnload or robot.wait):
            twist2 = Twist()
        pub.publish(twist)
        pub2.publish(twist2)

        rate.sleep() #sleep until the next time to publish
        

    #Send empty twist command to make sure robot stops
    twist = Twist()
    pub.publish(twist)

if __name__ == '__main__':
    try:
        potential()
    except rospy.ROSInterruptException:
        pass

