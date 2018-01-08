# -*- coding: utf-8 -*-
"""
Created on Mon Dec  4 12:13:50 2017

@author: yc
"""

import time
import thread
import threading
import RobotRaconteur as RR
from openravepy import *
import numpy as np
from OpenRAVEfunctions import *

RRN = RR.RobotRaconteurNode.s

class OpenRAVEObject(object):
	def __init__(self):
         print "Create OpenRAVE object"
         self._lock = threading.RLock()
         self.CollisionInit()
			
	def CollisionInit(self):
	     with self._lock:
	         self.cc = CollisionChecker(gui=True)
	         robot_pose = [0,0,0,1,0,0,0]
             self.T1 = np.dot(translation_matrix(robot_pose[0:3]), quaternion_matrix(robot_pose[3:7]))
             
             """ """
             self.T2 = np.dot(translation_matrix([0,0,0]), quaternion_matrix([1,0,0,0]))
             #self.T2 = np.dot(translation_matrix([1.2,0,1.2]), quaternion_matrix([0,0,0,1]))
	     
	     
	def CollisionReport(self,q1,q2,q3,q4,q5,q6):
         #with self._lock:
             #cc = CollisionChecker(gui=False)  
             
             #robot_pose = [0,0,0,1,0,0,0]
             #T1 = np.dot(translation_matrix(robot_pose[0:3]), quaternion_matrix(robot_pose[3:7]))
             #T2 = np.dot(translation_matrix([0,0,0]), quaternion_matrix([0,0,0,1]))
             #T2 = np.dot(translation_matrix([1,0,1.5]), quaternion_matrix([0,0,0,1]))
             #T3 = np.dot(translation_matrix([0,-2,0.35]), quaternion_matrix([0.5, 0.5, 0.5, 0.5]))
             
             tmp_array = (q1,q2,q3,q4,q5,q6)
             #T3 = np.dot(T1,np.dot(FK_Matrix2(tmp_array), quaternion_matrix([0.707, 0.707, 0, 0])))
             joints = { 'irb6640_185_280_Testbed' : tmp_array }
            
             collision_poi = { 'irb6640_185_280_Testbed' : self.T1 }
             #collision_env = { 'Walls'   : self.T2, 'box' : T3 }
             
             """ """
             collision_env = { 'Walls'   : self.T2}
             #collision_env = { 'ball'   : self.T2 }
             
             tmp_result = self.cc.check_safety(collision_poi, collision_env, joints)
             
             """ """
             #print 'Safe =',tmp_result
             #print 'Safe =',tmp_result[1], tmp_result[2], tmp_result[3] 
             
             #T3 = np.dot(translation_matrix([0,0,0]), quaternion_matrix([0,0,0,1]))
             #collision_poi = { 'irb6640_185_280_Testbed' : T1, 'box' : T3 }
             #time.sleep(0.1)
             #tmp = cc.check_safety(collision_poi, collision_env, joints)

             #cc = CollisionChecker(gui=False) 
             
             """ """
             return tmp_result[2], tmp_result[3]
     

	
def main():    
    RRN.UseNumPy = True
    
    # Create and Register Local Transport (names the node example.math)
    t1 = RR.LocalTransport()
    t1.StartServerAsNodeName("OpenRAVE_rr")
    RRN.RegisterTransport(t1)
    
    # Create and Register TCP Transport
    t2 = RR.TcpTransport()
    t2.EnableNodeAnnounce()
    t2.StartServer(1234)
    RRN.RegisterTransport(t2)
    
    # read in Service Definition File
    with open('OpenRAVE_rr.robdef','r') as f:
        service_def = f.read()
    
    # Register Service Definition
    RRN.RegisterServiceType(service_def)
    
    # Create instance of OpenRAVEObject object
    OpenRAVE_obj = OpenRAVEObject()
    
    # Register Service 'OpenRAVEObject' of type 'example.math.MathSolver'
    RRN.RegisterService("OpenRAVEObject","OpenRAVE_rr.OpenRAVEObject", OpenRAVE_obj)
       
    print "Connect to OpenRAVEObject at:"
    # address : port / node name / service
    print "tcp://localhost:1234/OpenRAVE_rr/OpenRAVEObject"
    raw_input('press enter to quit')
    
#    # THIS MUST BE HERE OR RISK SEGFAULT
#    RRN.Shutdown()
#
#    port = 10001       
#    t1 = RR.LocalTransport()
#    t1.StartServerAsNodeName("dobotRR")
#    RRN.RegisterTransport(t1)
#
#    t2 = RR.TcpTransport()
#    t2.EnableNodeAnnounce()
#    t2.StartServer(port)
#    RRN.RegisterTransport(t2)
    
#    my_dobot = DobotObject('COM4')
	

#    with open('dobotRR.robodef', 'r') as f:
#        service_def = f.read()
#    
#    RRN.RegisterServiceType(service_def)
#    RRN.RegisterService("dobotController", "dobotRR.DobotObject", my_dobot)
#    print "Conect string: tcp://localhost:" + str(port) + "/dobotRR/dobotController"
#    my_dobot.loop()



if __name__ == '__main__':
    main()
    
