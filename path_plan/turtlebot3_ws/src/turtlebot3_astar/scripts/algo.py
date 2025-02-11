#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt
import numpy as np
import operator
import random 

def check_node(node, clearance):
  offset = 5.1
  # Checking if point inside map
  if node[0] + clearance >= 10.1-offset or node[0] - clearance <= 0.1-offset  or node[1] + clearance >= 10.1-offset  or node[1] - clearance <= 0.1-offset :
    print('Sorry the point is out of bounds! Try again.')
    return False
  # Checking if point inside circles
  elif (node[0] - (3.1-offset) ) ** 2 + (node[1] - (2.1-offset)) ** 2 <= (1+clearance) ** 2 :
    print('Sorry the point is in the circle 1 obstacle space! Try again')
    return False
  elif (node[0] - (7.1-offset)) ** 2 + (node[1] - (2.1-offset)) ** 2 <= (1+clearance) ** 2 :
    print('Sorry the point is in the circle 2 obstacle space! Try again')
    return False
  elif (node[0] - (5.1-offset)) ** 2 + (node[1] - (5.1-offset)) ** 2 <= (1+clearance) ** 2 :
    print('Sorry the point is in the circle 3 obstacle space! Try again')
    return False
  elif (node[0] - (7.1-offset)) ** 2 + (node[1] - (8.1-offset)) ** 2 <= (1+clearance) ** 2 :
    print('Sorry the point is in the circle 4 obstacle space! Try again')
    return False
  # Checking if point inside squares
  elif node[0] + clearance >= 0.35-offset  and node[0] - clearance <=1.85-offset  and node[1] + clearance>= 4.35-offset  and node[1] - clearance< 5.85-offset :
    print('Sorry the point is in the square 1 obstacle space! Try again')
    return False
  elif node[0] + clearance >= 2.35-offset  and node[0] - clearance <= 3.85-offset  and node[1] + clearance>= 7.35-offset  and node[1] - clearance <= 8.85-offset :
    print('Sorry the point is in the square 2 obstacle space! Try again')
    return False
  elif node[0] + clearance >= 8.35-offset  and node[0] - clearance <= 9.85-offset  and node[1] + clearance>= 4.35-offset  and node[1] - clearance <= 5.85-offset :
    print('Node is:', node[0] + clearance >= 8.25)
    print('Sorry the point is in the square 3 obstacle space! Try again')
    return False
  else:
    return True

k = 0
visited_nodes = np.zeros((50,50,37))
valid_childs_dict = {}
explored = []
class Node():
  def __init__(self, start_node, goal_node, parent_node, clearance, rpm1, rpm2):
    self.start_node = start_node
    self.parent_node = parent_node
    self.clearance = clearance
    self.goal_node = goal_node
    self.rpm1 = rpm1
    self.rpm2 = rpm2
    self.weight = 10*max(self.rpm1, self.rpm2)

  def move(self,Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 0.033
    L = 0.160
    dt = 1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180

    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        linear_vel_x = 0.5*r * (UL + UR) * math.cos(Thetan)/dt
        linear_vel_y = 0.5*r * (UL + UR) * math.sin(Thetan)/dt
        ang_vel_z = ((r / L) * (UR - UL))/dt

    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, [linear_vel_x, linear_vel_y, ang_vel_z]

  def move1(self,node, cost):
    x_new, y_new, theta_new, vel = self.move(node[0], node[1], node[2], 0, self.rpm1)
    angle = (theta_new)%360
    new_node = [x_new, y_new, angle]
    cost2come = cost + 1.5*max(self.rpm1, self.rpm2)
    cost2go = self.weight*((self.goal_node[0]-new_node[0])**2 + (self.goal_node[1]-new_node[1])**2)**(1/2)
    total_cost = cost2come + cost2go
    new_node.append(total_cost)
    new_node.append(cost2come)
    new_node.append(vel)
    return new_node
    
  def move2(self,node, cost):
    x_new, y_new, theta_new, vel = self.move(node[0], node[1], node[2], self.rpm1, 0)
    angle = (theta_new)%360
    new_node = [x_new, y_new, angle]
    cost2come = cost + 1.5*max(self.rpm1, self.rpm2)
    cost2go = self.weight*((self.goal_node[0]-new_node[0])**2 + (self.goal_node[1]-new_node[1])**2)**(1/2)
    total_cost = cost2come + cost2go
    new_node.append(total_cost)
    new_node.append(cost2come)
    new_node.append(vel)
    return new_node
    
  def move3(self,node, cost):
    x_new, y_new, theta_new, vel = self.move(node[0], node[1], node[2], self.rpm1, self.rpm1)
    angle = (theta_new)%360
    new_node = [x_new, y_new, angle]
    cost2come = cost + 1*max(self.rpm1, self.rpm2)
    cost2go = self.weight*((self.goal_node[0]-new_node[0])**2 + (self.goal_node[1]-new_node[1])**2)**(1/2)
    total_cost = cost2come + cost2go
    new_node.append(total_cost)
    new_node.append(cost2come)
    new_node.append(vel)
    return new_node
    
  def move4(self,node, cost):
    x_new, y_new, theta_new, vel = self.move(node[0], node[1], node[2], 0, self.rpm2)
    angle = (theta_new)%360
    new_node = [x_new, y_new, angle]
    cost2come = cost + 1.5*max(self.rpm1, self.rpm2)
    cost2go = self.weight*((self.goal_node[0]-new_node[0])**2 + (self.goal_node[1]-new_node[1])**2)**(1/2)
    total_cost = cost2come + cost2go
    new_node.append(total_cost)
    new_node.append(cost2come)
    new_node.append(vel)
    return new_node
    
  def move5(self,node, cost):
    x_new, y_new, theta_new, vel = self.move(node[0], node[1], node[2], self.rpm2, 0)
    angle = (theta_new)%360
    new_node = [x_new, y_new, angle]
    cost2come = cost + 1.5*max(self.rpm1, self.rpm2)
    cost2go = self.weight*((self.goal_node[0]-new_node[0])**2 + (self.goal_node[1]-new_node[1])**2)**(1/2)
    total_cost = cost2come + cost2go
    new_node.append(total_cost)
    new_node.append(cost2come)
    new_node.append(vel)
    return new_node
    
  def move6(self,node, cost):
    x_new, y_new, theta_new, vel = self.move(node[0], node[1], node[2], self.rpm2, self.rpm2)
    angle = (theta_new)%360
    new_node = [x_new, y_new, angle]
    cost2come = cost + 1*max(self.rpm1, self.rpm2)
    cost2go = self.weight*((self.goal_node[0]-new_node[0])**2 + (self.goal_node[1]-new_node[1])**2)**(1/2)
    total_cost = cost2come + cost2go
    new_node.append(total_cost)
    new_node.append(cost2come)
    new_node.append(vel)
    return new_node
    
  def move7(self,node, cost):
    x_new, y_new, theta_new, vel = self.move(node[0], node[1], node[2], self.rpm1, self.rpm2)
    angle = (theta_new)%360
    new_node = [x_new, y_new, angle]
    cost2come = cost + 1.5*max(self.rpm1, self.rpm2)
    cost2go = self.weight*((self.goal_node[0]-new_node[0])**2 + (self.goal_node[1]-new_node[1])**2)**(1/2)
    total_cost = cost2come + cost2go
    new_node.append(total_cost)
    new_node.append(cost2come)
    new_node.append(vel)
    return new_node
    
  def move8(self,node, cost):
    x_new, y_new, theta_new, vel = self.move(node[0], node[1], node[2], self.rpm2, self.rpm1)
    angle = (theta_new)%360
    new_node = [x_new, y_new, angle]
    cost2come = cost + 1.5*max(self.rpm1, self.rpm2)
    cost2go = self.weight*((self.goal_node[0]-new_node[0])**2 + (self.goal_node[1]-new_node[1])**2)**(1/2)
    total_cost = cost2come + cost2go
    new_node.append(total_cost)
    new_node.append(cost2come)
    new_node.append(vel)
    return new_node

  def index(self, node):
    return (node[0], node[1], node[2])

  def child_generator(self, node, cost):
    valid_children = []
    n1 = self.move1(node, cost)
    n2 = self.move2(node, cost)
    n3 = self.move3(node, cost)
    n4 = self.move4(node, cost)
    n5 = self.move5(node, cost)
    n6 = self.move6(node, cost)
    n7 = self.move7(node, cost)
    n8 = self.move8(node, cost)
    childs = [n1,n2,n3,n4,n5,n6,n7,n8]
    childs_cost = {n1[3]:n1,n2[3]:n2,n3[3]:n3,n4[3]:n4, n5[3]:n5, n6[3]:n6,n7[3]:n7, n8[3]:n8}
    for cost in childs_cost.keys():
      if utils.check_node(childs_cost[cost], self.clearance) == True and visited_nodes[int(round(childs_cost[cost][0],1)/0.2)][int(round(childs_cost[cost][1],1)/0.2)][int(round(childs_cost[cost][2],1)/10)] == 0:
        valid_children.append((cost, childs_cost[cost], childs_cost[cost][4], self.index(childs_cost[cost]),node))
        valid_childs_dict[self.index(childs_cost[cost])] = [childs_cost[cost], node, self.index(node)]
        visited_nodes[int(round(childs_cost[cost][0],1)/0.2)][int(round(childs_cost[cost][1],1)/0.2)][int(round(childs_cost[cost][2],1)/10)] = 1
        
    return valid_children

  def astar(self):
    print('Started Search ... ')
    explored_nodes = [(0, self.start_node, self.index(self.start_node), self.parent_node)]
    explored.append(self.start_node)
    valid_childs_dict[self.index(self.start_node)] = [self.start_node, self.parent_node]
    cum_cost = 0
    itr = 0
    while len(explored_nodes) > 0:
      print('Explored Depth:', itr)
      min_cost_child = explored_nodes[0][1]
      explored_nodes.pop(0)
      child_costs= self.child_generator(min_cost_child, cum_cost)
      for child in child_costs:
        explored_nodes.append(child)
        explored.append(child)
      explored_nodes.sort(key=operator.itemgetter(0))
      cum_cost = explored_nodes[0][2]
      itr = itr+1
      if ((min_cost_child[0] - self.goal_node[0]) ** 2 + (min_cost_child[1] - self.goal_node[1]) ** 2) <= 0.25 ** 2:
        final_node_key =  (min_cost_child[0], min_cost_child[1], min_cost_child[2])
        print('Goal node found!')
        break 
    print('Started Backtracking ...')
    final_path= self.back_track(final_node_key)
    print('Backtracking complete!')
    return final_path, explored
        
  def back_track(self, node_ind):
    path = [valid_childs_dict[node_ind][0]]
    while node_ind != self.index(self.start_node):
      parent = valid_childs_dict[node_ind][1]
      path.insert(0, parent)
      node_ind = valid_childs_dict[node_ind][2]
    print('The path is:', path)
    return path
   
