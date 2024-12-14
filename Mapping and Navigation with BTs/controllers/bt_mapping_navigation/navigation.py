import py_trees
import numpy as np
import sys




class Navigation(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Navigation, self).__init__(name)
        
        self.robot=blackboard.read('robot')
        self.blackboard=blackboard
        self.name = name
        
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        self.motor_left = self.robot.getDevice('wheel_left_joint')
        self.motor_right = self.robot.getDevice('wheel_right_joint')
        self.motor_left.setPosition(float('Inf'))
        self.motor_right.setPosition(float('Inf'))
        
        self.marker = self.robot.getFromDef("marker").getField("translation")
        
        self.logger.debug(" %s [Navigation::setup()] " % self.name)
   
    def initialise(self):
        self.motor_left.setVelocity(0.0)
        self.motor_right.setVelocity(0.0)
        
        self.index = 0
        
        self.logger.debug("  %s [Navigation::initialise()]" % self.name)
        self.WP=self.blackboard.read('waypoints')
        print(self.name)
   
   
    def update(self):
        self.logger.debug("  %s [Navigation::update()]" % self.name)

        
        x_w = self.gps.getValues()[0] 
        y_w = self.gps.getValues()[1] 
        theta=np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
    
        rho = np.sqrt((x_w-self.WP[self.index][0])**2 + (y_w-self.WP[self.index][1])**2)
        alpha = np.arctan2(self.WP[self.index][1]-y_w, self.WP[self.index][0]-x_w)-theta
        
      
        if (alpha > np.pi):
            alpha = alpha - 2*np.pi  
        elif(alpha < -np.pi):
            alpha = alpha + 2*np.pi
    
    
        self.marker.setSFVec3f([*self.WP[self.index], 0.02])
        
        p1 = 5
        p2 = 4
     
        vL = -p1*alpha + p2*rho
        vR = p1*alpha + p2*rho
        
        vL *= 0.6
        vR *= 0.6
        
        vL = min(vL, 6.28)
        vR = min(vR, 6.28)
        vL = max(vL, -6.28)
        vR = max(vR, -6.28)
        
        self.motor_left.setVelocity(vL)
        self.motor_right.setVelocity(vR)
        
        
        if (rho<0.4):
            print("Reached ", self.index, len(self.WP))
            self.index = self.index + 1
            
            if self.index == len(self.WP):
            
                print("Reached last waypoint")
                self.feedback_message = "Last waypoint reached"
                
                #update the robot's current location value in the blackboard
                self.blackboard.write('current location', self.WP[len(self.WP)-1])
                print(f"current location => {self.blackboard.read('current location')}")
                
                #Check if the robot has reached the sink.
                if self.name == 'Move to sink':
                    print("Completed all tasks...stopping simulation")
                    self.blackboard.write('Completed all tasks', True)
                    self.motor_left.setVelocity(0.0)
                    self.motor_right.setVelocity(0.0)
                    sys.exit(0)

               
                return py_trees.common.Status.SUCCESS
                
            else:
                return py_trees.common.Status.RUNNING
                
            
                
        else:
            return py_trees.common.Status.RUNNING
            
            
    def terminate(self, new_status):
        pass
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    