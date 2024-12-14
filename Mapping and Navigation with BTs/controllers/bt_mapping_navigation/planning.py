from skimage.draw import line_nd, random_shapes
from heapq import heapify, heappush, heappop
from matplotlib import pyplot as plt
from collections import defaultdict
from collision import IsPathOpen
from scipy import signal
import numpy as np
import py_trees
import math
import sys
import ast


def world2map(x_w,y_w):
    
   x_w_origin = -2.16
   y_w_origin = 1.78
   p_x_origin = 0.0
   p_y_origin = 0.0
   
   x_w_end = 2.42
   y_w_end = -4.07
   p_x_end = 199.0
   p_y_end = 299.0
   
   px = int(( ((x_w - x_w_origin) * (p_x_end - p_x_origin)) / \
         (x_w_end - x_w_origin)) + p_x_origin)
         
   py = int(( ((y_w - y_w_origin) * (p_y_end - p_y_origin)) / \
         (y_w_end - y_w_origin)) + p_y_origin)
   
   px = min(px, 199)
   py = min(py, 299)
   px = max(px, 0)
   py = max(py, 0)
   
   return [px,py]


# A function that maps map coordinates to world coordinates
def map2world(p_x,p_y):
    
    
   p_x_origin = 0.0
   p_y_origin = 0.0
   x_w_origin = -2.16
   y_w_origin = 1.78
   
   p_x_end = 199.0
   p_y_end = 299.0
   x_w_end = 2.42
   y_w_end = -4.07

   
   x_w = ( ((p_x - p_x_origin) * (x_w_end - x_w_origin)) / \
         (p_x_end - p_x_origin)) + x_w_origin
         
   y_w = ( ((p_y - p_y_origin) * (y_w_end - y_w_origin)) / \
         (p_y_end - p_y_origin)) + y_w_origin
   
   
   return [x_w,y_w]
   
   
#A-star, called after the RRT has generated a graph.  
def getShortestPath(map, start, goal):

    plt.plot(start[1],start[0],'y*')
    plt.plot(goal[1],goal[0],'y*')
    
    def getNeighbors(u):
        neighbors = []
        for cand in map[u]:
            cost = np.sqrt((u[0]-cand[0])**2 + (u[1]-cand[1])**2)
            neighbors.append((round(cost, 4), cand)) 
        return neighbors
        
      
    queue = [(0, start)]
    visited = {start}
    parent = {}
    heapify(queue) 
    
    distances=defaultdict(lambda:float("inf"))
    distances[start]=0 
    
    while queue:
        (current_dist, v) = heappop(queue)
        visited.add(v)
        
        if v == goal:
            break
        
        for (cost_vu, u) in getNeighbors(v):
            if u not in visited:
            
                heuristic = np.sqrt((goal[0]-u[0])**2 + (goal[1]-u[1])**2)
                new_cost = distances[v] + cost_vu 
                
                if new_cost < distances[u]:
                    distances[u] = new_cost
                    heappush(queue, (new_cost + heuristic, u))
                    parent[u] = v
    
    key = goal
    path = []
    
    
    while key in parent.keys():
        key = parent[key]
        path.insert(0,key)
    path.append(goal)
    
    print("Path length => ", len(path))
   
    #write optimal path to path.txt file
    with open('path.txt', 'w', encoding='utf-8', newline='') as file:
        optimal_path = [str(x) for x in path]
        optimal_path = ",".join(optimal_path)
        optimal_path = str(path)
        print("Writing path to file")
        file.write(optimal_path)
    print("Done writing path to file")
    
    for p in path:
            plt.plot(p[1],p[0],'r.')
            plt.show()
            plt.pause(0.000001)
    plt.close()   
   
 
    


class Planning(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, goal):
        super(Planning, self).__init__(name)
        self.robot=blackboard.read('robot')
        self.blackboard=blackboard
        
        px, py = world2map(goal[0], goal[1])
        self.goal = (px, py)
        
        
        
    def setup(self):
         self.logger.debug("   %s [Planner::initialise()]" % self.name)
       
    
    def initialise(self):
        self.logger.debug("  %s [Planner::initialise()]" % self.name)
        self.Dq = 15 #the incremental distance with which to build the tree.
        self.K = 10000 #the maximal number of vertices in the RRT.
         
        start_map = self.blackboard.read('current location')
        px_start, py_start = tuple(world2map(start_map[0],start_map[1]))
        self.start = (px_start, py_start)
        
        self.Graph = {self.start:[]} #RRT graph.
        self.parent = {} #dictionary to store parents of each node
         
        
        self.map = np.load('cspace.npy')
        plt.imshow(self.map) # shows the map
        plt.ion() # turns 'interactive mode' on
        
        # puts a yellow asterisk at the start
        plt.plot(self.start[1],self.start[0], marker='*', color="yellow") 
        
        # puts a yellow asterisk at the goal    
        plt.plot(self.goal[1],self.goal[0], marker='*', color="green") 
   
        print("Running RRT and A-star")
    
    def update(self):
        k = 0 
        q_near = None #q_near initialized to None.
        q_new = None #q_new initialized to None.
        
        
        while (q_new != self.goal) and (k<self.K): 
                
                #Generate q_rand, a random configuration within the map boundaries. 
                q_rand = (np.random.randint(0,len(self.map)), \
                np.random.randint(0,len(self.map[0])))
              
                    
                #Initialize the initial distance between q_rand and the nodes 
                #in G to a very high number (float("inf")).
                shortest_dist = float("inf")
                
                #Loop over the nodes in G and find the closest node to q_rand.
                for q in self.Graph:
                    dist_q_to_qrand = np.sqrt( (q_rand[0]-q[0])**2 + (q_rand[1]-q[1])**2)
                    
                    if dist_q_to_qrand < shortest_dist:
                        shortest_dist = dist_q_to_qrand
                        q_near = q
                

                #Check if q_rand is closer than Dq to q_near
                if shortest_dist < self.Dq:
                    q_new = q_rand
        
                #If not, find the closes point to q_near in the direction of q_rand
                #Set q_new to be equal to that value.
                else:
                    q_diff = np.array(q_rand)-np.array(q_near)
                   
                    q_new = q_near + (( (q_diff)/np.linalg.norm(q_diff) ) * self.Dq)
                    q_new = tuple(q_new)
                    q_new = tuple(round(x, 1) for x in q_new)
    
                #Check if goal has been reached.    
                goal_dist = np.sqrt((q_new[0]-self.goal[0])**2+(q_new[1]-self.goal[1])**2)    
                if goal_dist<self.Dq:        
                    q_new=self.goal
                   
                
                
                #Check if the edge between q_near and q_new is collision free.
                collision_free = IsPathOpen(self.map,q_near,q_new)   
                if collision_free:
                    self.Graph[q_near].append(q_new) # Update neighbor list for q_near.
                    self.Graph[q_new] = [] # Add q_new to the graph.
                    self.Graph[q_new].append(q_near) # Add q_near to q_new's neighbor list.
                    self.parent[q_new] = q_near # Update parent dictionary to reflect that q_new's parent is q_near.
                    
                    plt.plot([q_new[1],self.parent[q_new][1]],[q_new[0],
                             self.parent[q_new][0]],'bo-', linewidth=2, markersize=3)
                    plt.show()
                    plt.pause(0.000001)

               
                k+=1 #Increment k
    
        #Run A-star on graph generated by RRT
        getShortestPath(self.Graph, self.start, self.goal) 
        
        
        #Read the shortest path generated by A-star and write the 
        #new waypoints (converted to world coordinates) to the blackboard.
        with open('path.txt', mode='r') as file:
            content = file.readline()
            print("Found the path file")
            if content:
                try:
                    path_waypoints = ast.literal_eval(content)
                    if isinstance(path_waypoints, list):
                        #print(f"path_waypoints => {path_waypoints}")
                        
                        
                        new_waypoints = []
                        for p in path_waypoints:
                           
                            (px, py) = map2world(p[0],p[1])
                            (px, py) = (round(px, 2), round(py, 2))
                            new_waypoints.append((px, py))
                            
                        #Update waypoints key in blackboard with the new path
                        #generated by A-star 
                        self.blackboard.write('waypoints', new_waypoints)
                        return py_trees.common.Status.SUCCESS  
                    else:
                        print("File does not contain a valid list of tuples.")
                        
                except (ValueError, SyntaxError):
                    print("File contains invalid Python expression.")
                    print(type(path_waypoints))
                
                
                file.close()
            else:
                print("path file is empty.")
                return py_trees.common.Status.RUNNING

                
     
    def terminate(self,new_status):
        pass
        
       

   
   
   
   
   
   
   
   
   
   
   
   