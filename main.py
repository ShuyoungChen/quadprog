# -*- coding: utf-8 -*-

import numpy as np
from numpy.linalg import inv
from scipy.linalg import logm, norm, sqrtm
import pygame
from ControlParams import *
from OpenRAVE_rr_server import *
import rpi_abb_irc5
import time
import timeit
from pyquaternion import Quaternion
from cvxopt import matrix, solvers

def robotParams():
    I3 = np.eye(3)
    ex = I3[:,0]
    ey = I3[:,1]
    ez = I3[:,2]
    
    h1 = ez
    h2 = ey
    h3 = ey
    h4 = ex
    h5 = ey
    h6 = ex
    P = np.array([[0,0,0], [0.32, 0, 0.78], [0, 0, 1.075], [0, 0, 0.2], [1.142, 0, 0], [0.2, 0, 0], [0.2,0,0]]).T
    q = np.zeros((6, 1))
    H = np.array([h1, h2, h3, h4, h5, h6]).T
    ttype = np.zeros((1, 6))
    """ """
    n = 6
    dq_bounds = np.array([[100,110], [90,90], [90,90], [170,190], [120,140], [190,235]]).T
    dq_bounds = dq_bounds*np.pi/180
    
    return ex,ey,ez,n,P,q,H,ttype,dq_bounds

def fwdkin(q,ttype,H,P,n):
    R=np.eye(3)
    p=np.zeros((3,1))
    
    for i in range(n):        
        h_i = H[:,i]
        Ri=np.eye(3)
        if ttype[0][i] == 0: 
            #rev
            pi = P[:,i].reshape(3, 1)
            p = p+np.dot(R, pi)
            Ri = rot(h_i,q[i])
            R = np.dot(R, Ri)
            R = Closest_Rotation(R)
        elif ttype[i] == 1: 
            #pris
            pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
            p = p+np.dot(R, pi)
        else: 
	        #default pris
	        pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
	        p = p+np.dot(R, pi)
  
    #End Effector T
    p=p+np.dot(R, P[0:3,n].reshape(3, 1))
    
    return R, p
    
# find closest rotation matrix 
# A=A*inv(sqrt(A'*A))   
def Closest_Rotation(R):
    R_n = np.dot(R, inv(sqrtm(np.dot(R.T, R))))
    
    return R_n

# ROT Rotate along an axis h by q in radius
def rot(h, q):
    h=h/norm(h)
    R = np.eye(3) + np.sin(q)*hat(h) + (1 - np.cos(q))*hat(h)**2
    
    return R

def hat(h):
    h_hat = np.array([[0, -h[2], h[1]], [h[2], 0, -h[0]], [-h[1], h[0], 0]])
    
    return h_hat
    
def fwdkin_alljoints(q, ttype, H, P, n):
    R=np.eye(3)
    p=np.zeros((3,1))
    RR = np.zeros((3,3,n+1))
    pp = np.zeros((3,n+1))

    for i in range(n):
        h_i = H[:,i] 
         
        if ttype[0][i] == 0:
        #rev
            pi = P[:,i].reshape(3, 1)
            p = p+np.dot(R,pi)
            Ri = rot(h_i,q[i])
            R = np.dot(R,Ri)
            R = Closest_Rotation(R)
        elif ttype[i] == 1: 
        #pris
            pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
            p = p+np.dot(R,pi)
        else: 
	    # default pris
	        pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
	        p = p+np.dot(R,pi)
 
        pp[:,i] = p.ravel()
        RR[:,:,i] = R
    
    # end effector T
    p=p+np.dot(R, P[0:3,n].reshape(3, 1))
    pp[:,n] = p.ravel()
    RR[:,:,n] = R
    
    return pp, RR

def Joint2Collision(Closest_Pt,pp):
    link_dist = []

    for i in range(6):
        link = pp[:,i+1]-pp[:,i]
        link = link/norm(link)
        pp2c = Closest_Pt - pp[:,i]
        
        link_dist.append(norm(pp2c - abs(np.dot(pp2c, link))*link))

    J2C_Joint = link_dist.index(min(link_dist)) + 1
    if(J2C_Joint==1):
        J2C_Joint=2
        
    return J2C_Joint

def getJacobian(q,ttype,H,P,n):
    num_joints = len(q)

    P_0_i = np.zeros((3,num_joints+1))
    R_0_i = np.zeros((3,3,num_joints+1))


    P_0_i,R_0_i=fwdkin_alljoints(q,ttype,H,P,n)
    
    P_0_T = P_0_i[:,num_joints]

    J = np.zeros((6,num_joints))
    
    for i in range(num_joints):
        if ttype[0][i] == 0:
            J[:,i] = np.hstack((np.dot(R_0_i[:,:,i],H[:,i]), np.dot(hat(np.dot(R_0_i[:,:,i], H[:,i])), (P_0_T - P_0_i[:,i]))))
    """ """
    
    return J

""" """
# return jacobian of the closest point on panel  
def getJacobian3(q,ttype,H,P,n, Closest_Pt, J2C_Joint):
    num_joints = len(q)

    P_0_i = np.zeros((3,num_joints+1))
    R_0_i = np.zeros((3,3,num_joints+1))


    P_0_i,R_0_i=fwdkin_alljoints(q,ttype,H,P,n)
    """  """
    
    P_0_T = Closest_Pt

    J = np.zeros((6,num_joints))
    
    for i in range(num_joints):
        if ttype[0][i] == 0:
            J[:,i] = np.hstack((np.dot(R_0_i[:,:,i],H[:,i]), np.dot(hat(np.dot(R_0_i[:,:,i], H[:,i])), (P_0_T - P_0_i[:,i]))))
    """ """
    J[:,J2C_Joint:7] = 0
    
    return J

# return jacobian of the closest point on robot        
def getJacobian2(q,ttype,H,P,n,Closest_Pt,J2C_Joint):

    num_joints = len(q)

    P_0_i,R_0_i = fwdkin_alljoints(q,ttype,H,P,n)

    P_0_T = P_0_i[:,num_joints]

    J = np.zeros((6,num_joints))

    for i in range(num_joints):
        if ttype[0][i] == 0:
            J[:,i] = np.hstack((np.dot(R_0_i[:,:,i], H[:,i]), np.dot(hat(np.dot(R_0_i[:,:,i], H[:,i])), (P_0_T - P_0_i[:,i]))))

    J[:,J2C_Joint:7] = 0
    link_c = P_0_i[:,J2C_Joint]-P_0_i[:,J2C_Joint-1]
    link_c = link_c/norm(link_c)
    
    P_0_tmp = P_0_i[:,J2C_Joint-1]+ abs(np.dot(Closest_Pt-P_0_i[:,J2C_Joint-1],link_c))*link_c
    
    return J,P_0_tmp

# convert a unit quaternion to angle/axis representation
def quat2axang(q):

    s = norm(q[0][1:4])
    if s >= 10*np.finfo(np.float32).eps:
        vector = q[0][1:4]/s
        theta = 2*np.arctan2(s,q[0][0])
    else:
        vector = np.array([0,0,1])
        theta = 0
    axang = np.hstack((vector,theta))
    
    return axang

def getqp_H(dq, J, vr, vp, er, ep):
    n = len(dq)
    H1 = np.dot(np.hstack((J,np.zeros((6,2)))).T,np.hstack((J,np.zeros((6,2)))))
    
    tmp = np.vstack((np.hstack((np.hstack((np.zeros((3,n)),vr)),np.zeros((3,1)))),np.hstack((np.hstack((np.zeros((3,n)),np.zeros((3,1)))),vp)))) 
    H2 = np.dot(tmp.T,tmp)

    H3 = -2*np.dot(np.hstack((J,np.zeros((6,2)))).T, tmp)
    H3 = (H3+H3.T)/2;
    
    tmp2 = np.vstack((np.array([0,0,0,0,0,0,np.sqrt(er),0]),np.array([0,0,0,0,0,0,0,np.sqrt(ep)])))
    H4 = np.dot(tmp2.T, tmp2)

    H = 2*(H1+H2+H3+H4)

    return H

def getqp_f(dq, er, ep):
    f = -2*np.array([0,0,0,0,0,0,er,ep]).reshape(8, 1)
    
    return f

def inequality_bound(h,c,eta,epsilon,e):
    sigma = np.zeros((h.shape))
    h2 = h - eta
    sigma[np.array(h2 >= epsilon)] = -np.tan(c*np.pi/2)
    sigma[np.array(h2 >= 0) & np.array(h2 < epsilon)] = -np.tan(c*np.pi/2/epsilon*h2[np.array(h2 >= 0) & np.array(h2 < epsilon)])
    sigma[np.array(h >= 0) & np.array(h2 < 0)] = -e*h2[np.array(h >= 0) & np.array(h2 < 0)]/eta
    sigma[np.array(h < 0)] = e
    
    return sigma

# quaternion multiply
def quatmultiply(q1, q0):
    w0, x0, y0, z0 = q0[0][0], q0[0][1], q0[0][2], q0[0][3]
    w1, x1, y1, z1 = q1[0][0], q1[0][1], q1[0][2], q1[0][3]
    
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64).reshape(1, 4)
                     

def main():
    #Init the joystick
    pygame.init()
    pygame.joystick.init()
    joy=pygame.joystick.Joystick(0)
    joy.init()
    clock=pygame.time.Clock()

    egm=rpi_abb_irc5.EGM()

    OpenRAVE_obj = OpenRAVEObject()

# Initialize Robot Parameters    
    ex,ey,ez,n,P,q_ver,H,ttype,dq_bounds = robotParams()

# Initialize Control Parameters
# initial joint angles
    
    """ """
    #q = np.zeros((6, 1))
    q = np.array([0,0,0,0,np.pi/2,0]).reshape(6, 1)
    
    R,pos = fwdkin(q,ttype,H,P,n)
 
    orien = Quaternion(matrix=R)
    orien = np.array([orien[0], orien[1], orien[2], orien[3]]).reshape(1, 4)

    pos_v = np.zeros((3, 1))
    ang_v = np.array([1,0,0,0])
    dq = np.zeros((int(n),1))
 
    # joint limits
    lower_limit = np.transpose(np.array([-170*np.pi/180, -65*np.pi/180, -np.pi, -300*np.pi/180, -120*np.pi/180, -2*np.pi]))
    upper_limit = np.transpose(np.array([170*np.pi/180, 85*np.pi/180, 70*np.pi/180, 300*np.pi/180, 120*np.pi/180, 2*np.pi]))
	
    # inequality constraints
    h = np.zeros((15, 1))
    sigma = np.zeros((13, 1))
    dhdq = np.vstack((np.hstack((np.eye(6), np.zeros((6, 1)), np.zeros((6, 1)))), np.hstack((-np.eye(6), np.zeros((6, 1)), np.zeros((6, 1)))), np.zeros((1, 8))))

    """be careful of dimension here"""
    view_port = np.array([-30, 30])
    
    # plot options
    axes_lim = np.array([-1.5, 3.2, -1.5, 3.2, -1.5, 3.2])
    
    # velocities
    w_t = np.zeros((3, 1))
    v_t = np.zeros((3, 1))
    
    # keyboard controls
    # define position and angle step
    inc_pos_v = 0.01 # m/s
    inc_ang_v = 0.5*np.pi/180 # rad/s

    # optimization params
    er = 0.05
    ep = 0.05
    epsilon = 0 # legacy param for newton iters
    
    # parameters for inequality constraints
    c = 0.5
    eta = 0.1
    epsilon_in = 0.1
    E = 0.01
    
    Ke = 1
    
    # create a handle of these parameters for interactive modifications
    obj = ControlParams(ex,ey,ez,n,P,H,ttype,dq_bounds,q,dq,pos,orien,pos_v,ang_v.reshape(1, 4),w_t,v_t,epsilon,view_port,axes_lim,inc_pos_v,inc_ang_v,0,er,ep,0)

    dt = 0
    counter = 0
    
    while not obj.params['controls']['stop']:

        # Loop reading the joysticks
        for event in pygame.event.get():
            pass

        if counter != 0:
            toc = timeit.default_timer()
            dt = toc - tic
            
        tic = timeit.default_timer()

        counter = counter + 1


        if counter != 0:
            obj.params['controls']['q'] = obj.params['controls']['q'] + obj.params['controls']['dq']*dt*0.1
            
            res, state = egm.receive_from_robot(0.01)

            if res:
                a = np.array(state.joint_angles)
                a = a * 180 / np.pi
                print "Joints: " + str(a)
                egm.send_to_robot([float(x)*180/np.pi for x in obj.params['controls']['q']])
                print "Target Joints: " + str([float(x)*180/np.pi for x in obj.params['controls']['q']])

            [pp,RR]=fwdkin_alljoints(obj.params['controls']['q'],ttype,H,P,n)

            # parameters for qp
            obj.params['controls']['pos'] = pp[:, -1]

            orien_tmp = Quaternion(matrix=RR[:, :, -1])
            obj.params['controls']['orien'] = np.array([orien_tmp[0], orien_tmp[1], orien_tmp[2], orien_tmp[3]]).reshape(1, 4)
            
            """ """
            Closest_Pt, Closest_Pt_env = OpenRAVE_obj.CollisionReport(obj.params['controls']['q'][0],obj.params['controls']['q'][1],obj.params['controls']['q'][2],obj.params['controls']['q'][3],obj.params['controls']['q'][4],obj.params['controls']['q'][5])
            
            J2C_Joint = Joint2Collision(Closest_Pt, pp)
            print Closest_Pt, J2C_Joint
            J_eef = getJacobian(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'])
            
            if (J2C_Joint < 4):
                J,p_0_tmp = getJacobian2(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'],Closest_Pt,J2C_Joint)
            
            else:            
                J = getJacobian3(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'], Closest_Pt,J2C_Joint)
            
            x = joy.get_axis(0)
            if (abs(x) < .2):
                x = 0
            else:
                x = (abs(x) - .2) / .8 * cmp(x, 0)

            # control of linear velocity
            b1 = joy.get_button(0)
            b2 = joy.get_button(1)
            b3 = joy.get_button(2)

            # control of angular velocity
            b4 = joy.get_button(3)
            b5 = joy.get_button(4)
            b6 = joy.get_button(5)
        
            # emergency stop
            b9 = joy.get_button(8)
        
            if (b9 == 1):
                print 'robot stopped'
                obj.params['controls']['pos_v'] = np.array([0,0,0]).reshape(3, 1)
                obj.params['controls']['ang_v'] = np.array([1,0,0,0]).reshape(1, 4)
      
            button = [x, b1, b2, b3, b4, b5, b6]
            func_xbox(button, obj)
            
            # update joint velocities
            axang = quat2axang(obj.params['controls']['ang_v'])

            # desired rotational velocity
            vr = axang[3]*axang[0:3]
            
            # desired linear velocity
            V_desired = obj.params['controls']['pos_v']
                        
            H = getqp_H(obj.params['controls']['dq'], J_eef, vr.reshape(3, 1), obj.params['controls']['pos_v'], obj.params['opt']['er'], obj.params['opt']['ep']) 
            f = getqp_f(obj.params['controls']['dq'],obj.params['opt']['er'], obj.params['opt']['ep'])
            
            H = matrix(H, tc='d')
            f = matrix(f, tc='d')
            
            # bounds for qp
            if obj.params['opt']['upper_dq_bounds']:
                bound = obj.params['defi']['dq_bounds'][1, :]
            else:
                bound = obj.params['defi']['dq_bounds'][0, :]

            LB = np.vstack((-0.1*bound.reshape(6, 1),0,0))
            UB = np.vstack((0.1*bound.reshape(6, 1),1,1))
            LB = matrix(LB, tc = 'd')
            UB = matrix(UB, tc = 'd')
                    
            # inequality constrains A and b
            h[0:6] = obj.params['controls']['q'] - lower_limit.reshape(6, 1)
            h[6:12] = upper_limit.reshape(6, 1) - obj.params['controls']['q']
            
            dx = Closest_Pt_env[0] - Closest_Pt[0]
            dy = Closest_Pt_env[1] - Closest_Pt[1]
            dz = Closest_Pt_env[2] - Closest_Pt[2]
            
            """ """
            dist = np.sqrt(dx**2 + dy**2 + dz**2)
            
            # derivative of dist w.r.t time
            der = np.array([dx*(dx**2 + dy**2 + dz**2)**(-0.5), dy*(dx**2 + dy**2 + dz**2)**(-0.5), dz*(dx**2 + dy**2 + dz**2)**(-0.5)])

            """ """
            h[12] = dist - 0.005 #0.005
            """ """ """ """
            #dhdq[12, 0:6] = np.dot(-der.reshape(1, 3), J_eef2[3:6,:])
            dhdq[12, 0:6] = np.dot(-der[None, :], J[3:6,:])
            
            sigma[0:12] =inequality_bound(h[0:12], c, eta, epsilon_in, E)
            sigma[12] = inequality_bound(h[12], c, eta, epsilon_in, E)           
            
            A = -dhdq
            b = -sigma
            
            A = matrix([matrix(A, tc='d'), matrix(np.eye(8), tc='d'), matrix(-np.eye(8), tc='d')])
           
            b = matrix([matrix(b, tc='d'), UB, -LB])
            
            """ """
            
            solvers.options['show_progress'] = False
            
            sol = solvers.qp(H,f,A,b)
            dq_sln = sol['x']
            
            b7 = joy.get_button(6)
            b8 = joy.get_button(7)
            
            # set desired eef position and orientation
            if (b7 == 1 and b8 == 1):
                x_des = pp[:, -1]
                R_des = RR[:,:,-1]
                print 'desired position set'
                print 'desired position set'
            
            # it seems that the equality constraints works better when close to the obstacle
            # equality constraints for maintaining end-effector position (pure rotation)    
            if (b7 == 1):
                print 'pure rotational movement'
                A_eq_pos = np.hstack((J_eef[3:6,:], np.zeros((3, 2))))
                b_eq_pos = 0.05*Ke*(x_des - pp[:, -1])
                
                A_eq_pos = matrix(A_eq_pos, tc  = 'd')
                b_eq_pos = matrix(b_eq_pos, (3, 1))
                
                dq_sln = solvers.qp(H,f,A,b,A_eq_pos,b_eq_pos)['x']
                
            # equality constraints for maintaining end-effector orientation (pure translation)
            if (b8 == 1):
                print 'pure translational movement'
                A_eq = np.hstack((J_eef[0:3,:], np.zeros((3, 2))))            
                w_skew = logm(np.dot(RR[:,:,-1],R_des.T))
                w = np.array([w_skew[2, 1], w_skew[0, 2], w_skew[1, 0]])
                b_eq = -0.01*Ke*w
                
                A_eq = matrix(A_eq, tc  = 'd')
                b_eq = matrix(b_eq, (3, 1))
                
                dq_sln = solvers.qp(H,f,A,b,A_eq,b_eq)['x']
            
            if len(dq_sln) < obj.params['defi']['n']:
                obj.params['controls']['dq'] = np.zeros((6,1))
                V_scaled = 0
                print 'No Solution'
            else:
                obj.params['controls']['dq'] = dq_sln[0: int(obj.params['defi']['n'])]
                V_scaled = dq_sln[-1]*V_desired
                vr_scaled = dq_sln[-2]*vr.reshape(3,1)
         
            V_linear = np.dot(J_eef[3:6,:], obj.params['controls']['dq'])
            V_rot = np.dot(J_eef[0:3,:], obj.params['controls']['dq'])
                           
            print '------Scaled desired linear velocity------'
            print V_scaled
                       
            print '------Real linear velocity by solving quadratic programming------'
            print V_linear
            
            print '------Scaled desired angular velocity------'
            print vr_scaled
            
            print '------Real angular velocity by solving quadratic programming------'
            print V_rot
           
    pygame.quit()


def func_xbox(button, obj):

    if (button[1] == 1):
        obj.params['controls']['pos_v'] = obj.params['controls']['pos_v'] + np.matrix([button[0]*obj.params['keyboard']['inc_pos_v'],0,0]).T
    if (button[2] == 1):
        obj.params['controls']['pos_v'] = obj.params['controls']['pos_v'] + np.matrix([0,button[0]*obj.params['keyboard']['inc_pos_v'],0]).T
    if (button[3] == 1):
        obj.params['controls']['pos_v'] = obj.params['controls']['pos_v'] + np.matrix([0,0,button[0]*obj.params['keyboard']['inc_pos_v']]).T
        
    # wx, wy, wz       
    if (button[4] == 1):
        obj.params['controls']['ang_v'] = quatmultiply(np.array([np.cos(obj.params['keyboard']['inc_ang_v']/2), button[0]*np.sin(obj.params['keyboard']['inc_ang_v']/2), 0, 0]).reshape(1, 4), obj.params['controls']['ang_v'])
    if (button[5] == 1):
        obj.params['controls']['ang_v'] = quatmultiply(np.array([np.cos(obj.params['keyboard']['inc_ang_v']/2), 0, button[0]*np.sin(obj.params['keyboard']['inc_ang_v']/2), 0]).reshape(1, 4), obj.params['controls']['ang_v'])
    if (button[6] == 1):
        obj.params['controls']['ang_v'] = quatmultiply(np.array([np.cos(obj.params['keyboard']['inc_ang_v']/2), 0, 0, button[0]*np.sin(obj.params['keyboard']['inc_ang_v']/2)]).reshape(1, 4), obj.params['controls']['ang_v'])
    
        
if __name__ == '__main__':
    main()
    
