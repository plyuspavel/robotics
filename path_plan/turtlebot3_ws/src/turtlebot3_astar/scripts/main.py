#!/usr/bin/env python3

import time
import matplotlib.pyplot as plt
import numpy as np
import algo
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def main():

  rospy.init_node('turtlebot3_vel_publisher', anonymous=True)
  velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  vel_msg = Twist()

  time.sleep(2)
  print('')
  print("************************* Let's move the turtlebot! *******************************")
  print('')
  clearance = 0.2
  start_point = eval(input('Enter the start coordinates for the robot in this format - [X_coord, Y_coord, Theta]:'))
  while not check_node(start_point, clearance):
    start_point = eval(input('Please enter the start coordinates in this format - [X_coord, Y_coord, Theta]:'))
  goal_point = eval(input('Please enter the goal coordinates of the robot in this format - [X_coord, Y_coord]:'))
  while not check_node(goal_point, clearance):
    goal_point = eval(input('Please enter the goal coordinates of the robot in this format - [X_coord, Y_coord]:'))

  rpm = eval(input('Please enter the RPM for both the wheels in this format - [RPM1,RPM2]:'))

  robot_radius = 0.1
  s1 = algo.Node(start_point, goal_point, [0,0], robot_radius+clearance, rpm[0], rpm[1])
  path, explored = s1.astar()

  # Plotting the explored nodes and final path
  points1x = []
  points1y = []
  points2x = []
  points2y = []
  points3x = []
  points3y = []
  points4x = []
  points4y = []
  
  for point in range(1,len(explored)):
    points1x.append(explored[point][4][0])
    points1y.append(explored[point][4][1])
    points2x.append(explored[point][1][0]-(explored[point][4][0]))
    points2y.append(explored[point][1][1]-(explored[point][4][1]))

  print('Path length:',len(path))
  for point in range(1,len(path)):
    if point < len(path):
      vel_msg.linear.x = (path[point][5][0]**2 + path[point][5][1]**2)**(1/2)
      vel_msg.linear.y = 0
      vel_msg.linear.z = 0
      vel_msg.angular.x = 0
      vel_msg.angular.y = 0
      vel_msg.angular.z = path[point][5][2]
      velocity_publisher.publish(vel_msg)
      print('Point:', point)
      now = rospy.get_rostime()
      print('ROS Time:', now.secs)
      time.sleep(1)
      points3x.append(path[point][0])
      points3y.append((path[point][1]))
      points4x.append((path[point+1][0])-(path[point][0]))
      points4y.append((path[point+1][1])-(path[point][1]))

  vel_msg.linear.x = 0
  vel_msg.linear.y = 0
  vel_msg.linear.z = 0
  vel_msg.angular.x = 0
  vel_msg.angular.y = 0
  vel_msg.angular.z = 0
  velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
  main()
