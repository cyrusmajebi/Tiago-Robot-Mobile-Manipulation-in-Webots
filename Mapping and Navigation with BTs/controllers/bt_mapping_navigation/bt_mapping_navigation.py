from matplotlib import pyplot as plt
from os.path import exists 
from scipy import signal

import numpy as np
import py_trees
import ast
import sys
import os

#imports the necessary classes from the py_trees library
from py_trees.composites import Sequence, Parallel, Selector
from controller import Robot, Supervisor


from navigation import Navigation #imports the Navigation class from navigation.py
from planning import Planning #imports the Planning (RRT & A-star) from planning.py
from mapping import Mapping #imports the Mapping class from mapping.py


#robot instance (in supervisor mode)
robot = Supervisor()


#set the world timestep for the simulation.
timestep = int(robot.getBasicTimeStep())


#Initial set of waypoints
WP = [(0.61,-0.61), (0.61,-2.91), (-0.72,-3.36), 
      (-1.83,-2.66), (-1.72, -1.61), (-1.56,-0.32), 
      (-0.55, 0.55), (-1.82, -0.06), 
      (-1.72, -1.61), (-1.83,-2.66), (-0.72,-3.36),
      (0.47,-2.91), (0.61,-0.61), (0,0)]
      


#Create blackboard class
class Blackboard:
    def __init__(self):
        self.data = {}
    def write(self, key, value):
        self.data[key] = value
    def read(self, key):
        return self.data.get(key)


current_location = (0.0, 0.0) #initial location of the robot in world coordinates
blackboard = Blackboard() #blackboard instance
blackboard.write('robot', robot) #write the robot to the blackboard
blackboard.write('waypoints', WP) #Write initial set of waypoints.


#creates a key in the blackboard to store the robot's current location
blackboard.write('current location', current_location)

#creates a key in the blackboard that checks if the robot has completed
#all taks.
blackboard.write('Completed all tasks', False)


#Class to check if a map exists
class DoesMapExist(py_trees.behaviour.Behaviour):
    def update(self):
        file_exists = exists('cspace.npy')
        print("Checking for map")
        if (file_exists):
            print("Map already exists")
            blackboard.write('map exists', True)
            return py_trees.common.Status.SUCCESS
        else:
            print("Map does not exist")
            blackboard.write('map exists', False)
            return py_trees.common.Status.FAILURE



"""behaviour tree instance (py_tree) consisting of a Sequence node which in turn
contains a Selector node as one of its child nodes. The selector node
checks for a map and generates one if it doesn't exist."""
tree = Sequence("Main", children= [
           Selector("Does map exist?", children=[
             DoesMapExist("Test for map"),
             Parallel("Mapping", policy=py_trees.common.ParallelPolicy.SuccessOnOne(), 
             children=[
                 Mapping("Map the environment", blackboard),
                 Navigation("Move around the table", blackboard)
                 ])
            ], memory=True),
            
            
            #Plan a path to the lower left corner with an RRT and A-star
            Planning("Compute path to lower left corner", blackboard, (-1.46, -3.12)),
            #Move the robot to the lower left corner
            Navigation("Move to lower left corner", blackboard),
            
            #Plan a path to the sink with an RRT and A-star
            Planning("Compute path to sink", blackboard, (0.88, 0.57)),
            #Move the robot to the sink.
            Navigation("Move to sink", blackboard), 
        
                       
        ])
 
 
tree.setup_with_descendants() 
 
 
#main controller loop
while robot.step(timestep) != -1:

    
    tree.tick_once()
    pass












