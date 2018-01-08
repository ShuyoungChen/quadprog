# -*- coding: utf-8 -*-

from oct2py import octave
from oct2py import Oct2Py
import numpy as np
from numpy.linalg import inv
from scipy.linalg import logm, norm, sqrtm
import pygame
from ControlParams import *
from OpenRAVE_rr_server import *
import rpi_abb_irc5
import time
from pyquaternion import Quaternion
from cvxopt import matrix, solvers

def robotParams():
    #function [ex,ey,ez,n,P,q,H,type,dq_bounds] = robotParams()
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
            pi = P[:,i][:, None]
            p = p+np.dot(R, pi)
            Ri = rot(h_i,q[i])
            R = np.dot(R, Ri)
            R = Closest_Rotation(R)
        elif ttype[i] == 1: 
            #pris
            pi = (P[:,i]+q[i]*h_i)[:, None]
            p = p+np.dot(R, pi)
        else: 
	        #default pris
	        pi = (P[:,i]+q[i]*h_i)[:, None]
	        p = p+np.dot(R, pi)
  
    #End Effector T
    p=p+np.dot(R, P[0:3,n][:, None])
    return R, p
    
# find closest rotation matrix 
# A=A*inv(sqrt(A'*A))   
def Closest_Rotation(R):
    R_n = np.dot(R, inv(sqrtm(np.dot(R.T, R))))
    return R_n

def rot(h, q):
    # ROT Rotate along an axis h by q in radius.
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
            pi = P[:,i][:, None]
            p = p+np.dot(R,pi)
            Ri = rot(h_i,q[i])
            R = np.dot(R,Ri)
            R = Closest_Rotation(R)
        elif ttype[i] == 1: 
        #pris
            pi = (P[:,i]+q[i]*h_i)[:, None]
            p = p+np.dot(R,pi)
        else: 
	    # default pris
	        pi = (P[:,i]+q[i]*h_i)[:, None]
	        p = p+np.dot(R,pi)
 
        pp[:,i] = p.ravel()
        RR[:,:,i] = R
    
    # end effector T
    p=p+np.dot(R, P[0:3,n][:, None])
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
    
    return J
    
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

def quat2axang(q):
    # convert a unit quaternion to angle/axis representation
    s = norm(q[1:4])
    if s >= 10*np.finfo(np.float32).eps:
        vector = q[2:4]/s
        theta = 2*np.arctan2(s,q[0])
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
    f = -2*np.array([0,0,0,0,0,0,er,ep])[:, None]

    return f

def inequality_bound(h,c,eta,epsilon,e):
    sigma = np.zeros((h.shape))
    h2 = h - eta
    sigma[np.array(h2 >= epsilon)] = -np.tan(c*np.pi/2)
    sigma[np.array(h2 >= 0) & np.array(h2 < epsilon)] = -np.tan(c*np.pi/2/epsilon*h2[np.array(h2 >= 0) & np.array(h2 < epsilon)])
    sigma[np.array(h >= 0) & np.array(h2 < 0)] = -e*h2[np.array(h >= 0) & np.array(h2 < 0)]/eta
    sigma[np.array(h < 0)] = e
    return sigma


def main():
    # Add .m files to path
    #octave.addpath('/home/Shuyang/Downloads/update/Velocity_Control_Constrained_ABB_OpenRAVE')

    #Init the joystick
    pygame.init()
    pygame.joystick.init()
    joy=pygame.joystick.Joystick(0)
    joy.init()
    clock=pygame.time.Clock()

    egm=rpi_abb_irc5.EGM()

    OpenRAVE_obj = OpenRAVEObject()
# Initialize Robot Parameters
    
    #[ex,ey,ez,n,P,q_nouse,H,ttype,dq_bounds] = octave.feval('robotParams', nout=9)
    
    ex,ey,ez,n,P,q_ver,H,ttype,dq_bounds = robotParams()

# Initialize Control Parameters
# initial joint angles
    q = np.zeros((6, 1))
    #[R,pos] = octave.feval('fwdkin', q,ttype,H,P,n, nout=2)
    
    R,pos = fwdkin(q,ttype,H,P,n)
 
    orien = Quaternion(matrix=R)
    orien = np.array([orien[0], orien[1], orien[2], orien[3]])[None, :]
    #orien = octave.feval('R2q', R).T

    pos_v = np.zeros((3, 1))
    ang_v = np.array([1,0,0,0])
    dq = np.zeros((int(n),1))
    # desired eef orientation
    R_des = R
    
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
    inc_ang_v = 0.2*np.pi/180 # rad/s

    # optimization params
    er = 0.05
    ep = 0.05
    epsilon = 0 # legacy param for newton iters
    
    # parameters for inequality constraints
    c = 0.5
    eta = 0.1
    epsilon_in = 0.15
    E = 0.001
    
    Ke = 1
    
    # create a handle of these parameters for interactive modifications
    obj = ControlParams(ex,ey,ez,n,P,H,ttype,dq_bounds,q,dq,pos,orien,pos_v,ang_v[None, :],w_t,v_t,epsilon,view_port,axes_lim,inc_pos_v,inc_ang_v,0,er,ep,0)

    dt = 0
    counter = 0
    
    while not obj.params['controls']['stop']:

        # Loop reading the joysticks
        for event in pygame.event.get():
            pass


        if counter != 0:
            dt = octave.toc(t_start)

        """ """
        #print dt
        t_start = octave.tic()
        counter = counter + 1


        if counter != 0:
            obj.params['controls']['q'] = obj.params['controls']['q'] + obj.params['controls']['dq']*dt*0.1
            
            res, state = egm.receive_from_robot(0.01)
            #print res
            #time.sleep(0.1)
            if res:
                a = np.array(state.joint_angles)
                a = a * 180 / np.pi
                print "Joints: " + str(a)
                egm.send_to_robot([float(x)*180/np.pi for x in obj.params['controls']['q']])
                print "Target Joints: " + str([float(x)*180/np.pi for x in obj.params['controls']['q']])

            [pp,RR]=fwdkin_alljoints(obj.params['controls']['q'],ttype,H,P,n)
            #[pp,RR] = octave.feval('robot_3d', obj.params['controls']['q'], nout=2)

            # pseudo obstacles
            x1 = 1.2
            y1 = 0
            z1 = 1.2
            radius = 0.2
            #octave.figure()
            #octave.feval('dispObstacles', 'sphere', radius,0,  x1, y1, z1, [0.8, 0.8, 0.8])


            # parameters for qp
            obj.params['controls']['pos'] = pp[:, -1]
            
            orien_tmp = Quaternion(matrix=RR[:, :, -1])
            obj.params['controls']['orien'] = np.array([orien_tmp[0], orien_tmp[1], orien_tmp[2], orien_tmp[3]])[None, :]
            #obj.params['controls']['orien'] = octave.feval('R2q', RR[:, :, -1]).T
            
            """ """
            Closest_Pt, Closest_Pt_env = OpenRAVE_obj.CollisionReport(obj.params['controls']['q'][0],obj.params['controls']['q'][1],obj.params['controls']['q'][2],obj.params['controls']['q'][3],obj.params['controls']['q'][4],obj.params['controls']['q'][5])
            #octave.feval('dispObstacles', 'sphere', 0.05,0, Closest_Pt[0], Closest_Pt[1], Closest_Pt[2], [1,0,0])
            
            J2C_Joint = Joint2Collision(Closest_Pt, pp)
            
            #J2C_Joint = octave.feval('Joint2Collision', Closest_Pt[:, None], pp)
            #[J,p_0_tmp] = octave.feval('getJacobian2', obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'],Closest_Pt[:, None],J2C_Joint, nout=2)
            #J_eef = octave.feval('getJacobian', obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'])
            J,p_0_tmp = getJacobian2(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'],Closest_Pt,J2C_Joint)
            J_eef = getJacobian(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'])
            
            #axang = octave.feval('quat2axang', obj.params['controls']['ang_v'])
            axang = quat2axang(obj.params['controls']['ang_v'])

            vr = axang[3]*axang[0:3]
            
            H = getqp_H(obj.params['controls']['dq'], J_eef, vr[:, None], obj.params['controls']['pos_v'], obj.params['opt']['er'], obj.params['opt']['ep']) 
            #H = octave.feval('getqp_H', obj.params['controls']['dq'], J_eef, vr[:, None], obj.params['controls']['pos_v'], obj.params['opt']['er'], obj.params['opt']['ep'])
            
            #f = octave.feval('getqp_f', obj.params['controls']['dq'],obj.params['opt']['er'], obj.params['opt']['ep'])
            f = getqp_f(obj.params['controls']['dq'],obj.params['opt']['er'], obj.params['opt']['ep'])

            # equality constraints
            A_eq = np.hstack((J_eef[0:3,:], np.zeros((3, 2))))
            w_skew = logm(np.dot(RR[:,:,-1],R_des.T))
            w = np.array([w_skew[2, 1], w_skew[0, 2], w_skew[1, 0]])
            b_eq = np.transpose(-Ke*w)

            # inequality constrains A and b
            h[0:6] = obj.params['controls']['q'] - lower_limit[:, None]
            h[6:12] = upper_limit[:, None] - obj.params['controls']['q']
            
            """ """
            #dx = x1 - Closest_Pt[0]
            #dy = y1 - Closest_Pt[1]
            #dz = z1 - Closest_Pt[2]
            
            dx = Closest_Pt_env[0] - Closest_Pt[0]
            dy = Closest_Pt_env[1] - Closest_Pt[1]
            dz = Closest_Pt_env[2] - Closest_Pt[2]
            
            """ """
            dist = np.sqrt(dx**2 + dy**2 + dz**2)
            #dist = np.sqrt(dx**2 + dy**2 + dz**2) - radius
            
            # derivative of dist w.r.t time
            der = np.array([dx*(dx**2 + dy**2 + dz**2)**(-0.5), dy*(dx**2 + dy**2 + dz**2)**(-0.5), dz*(dx**2 + dy**2 + dz**2)**(-0.5)])

            """ """
            h[12] = dist - 0.1
            
            dhdq[12, 0:6] = np.dot(-der[None, :], J[3:6,:])
            
            #sigma[0:12] = octave.feval('inequality_bound', h[0:12], c, eta, epsilon_in, E)
            sigma[0:12] =inequality_bound(h[0:12], c, eta, epsilon_in, E)
            #sigma[12] = octave.feval('inequality_bound', h[12], c, eta, epsilon_in, E)
            sigma[12] = inequality_bound(h[12], c, eta, epsilon_in, E)           
            
            A = -dhdq
            b = -sigma

            # bounds for qp
            if obj.params['opt']['upper_dq_bounds']:
                bound = obj.params['defi']['dq_bounds'][1, :]
            else:
                bound = obj.params['defi']['dq_bounds'][0, :]

            LB = np.vstack((-0.1*bound[:, None],0,0))
            UB = np.vstack((0.1*bound[:, None],1,1))
            LB = matrix(LB, tc = 'd')
            UB = matrix(UB, tc = 'd')
            
            H = matrix(H, tc='d')
            f = matrix(f, tc='d')
          
            A = matrix([matrix(A, tc='d'), matrix(np.eye(8), tc='d'), matrix(-np.eye(8), tc='d')])
           
            b = matrix([matrix(b, tc='d'), UB, -LB])
           
            A_eq = matrix(A_eq, tc  = 'd')
            b_eq = matrix(b_eq, (3, 1))
            # quadratic programming
            #options = octave.optimset('Display', 'off')
            #dq_sln = octave.quadprog(H,f,A,b,A_eq,b_eq,LB,UB,np.vstack((obj.params['controls']['dq'],0,0)),options)
            
            """
            It seems that the cvxot quadprog solver has strange solution when close to obstacles, jump between plus and minus.
            """
            sol = solvers.qp(H,f,A,b,A_eq,b_eq)
            dq_sln = sol['x']
            
            # update joint velocities
            V_desired = obj.params['controls']['pos_v']
            
            if len(dq_sln) < obj.params['defi']['n']:
                obj.params['controls']['dq'] = np.zeros((6,1))
                V_scaled = 0
                print 'No Solution'
            else:
                obj.params['controls']['dq'] = dq_sln[0: int(obj.params['defi']['n'])]
                V_scaled = dq_sln[-1]*V_desired
                #V_scaled = np.asscalar(dq_sln[-1])*V_desired
         
            V_now = np.dot(J_eef[3:6,:], obj.params['controls']['dq'])
            """
            print V_desired
            """
            print '------------'
            print V_scaled
            
            print '------------'
            print V_now
            

            """
            if norm(V_now)>=1e-10 and norm(V_scaled)>=1e-10:
                direrr =  1-abs(np.dot(V_now.T/norm(V_now),V_scaled/norm(V_scaled)))
                if direrr<1-np.cos(1*np.pi/180):
                    print 'following direction'
                elif direrr<1-np.cos(5*np.pi/180):
                    print 'following direction imprecisely, dir mismatch(degrees):' + repr(np.arccos((1-direrr)*np.pi/180))
                else:
                    print 'Wrong direction, dir mismatch(degrees):' + repr(np.arccos((1-direrr)*np.pi/180))
            else:
                print 'Zero Velocity Occurs'
            """

        x = joy.get_axis(0)
        if (abs(x) < .2):
            x = 0
        else:
            x = (abs(x) - .2) / .8 * cmp(x, 0)

        b1 = joy.get_button(0)
        b2 = joy.get_button(1)
        b3 = joy.get_button(2)

        button = [x, b1, b2, b3]
        func_xbox(button, obj)
        
        # Limit to 20 frames per second
        #clock.tick(20)



    pygame.quit()

def func_xbox(button, obj):

    if (button[1] == 1):
        obj.params['controls']['pos_v'] = obj.params['controls']['pos_v'] + np.matrix([button[0]*obj.params['keyboard']['inc_pos_v'],0,0]).T
    if (button[2] == 1):
        obj.params['controls']['pos_v'] = obj.params['controls']['pos_v'] + np.matrix([0,button[0]*obj.params['keyboard']['inc_pos_v'],0]).T
    if (button[3] == 1):
        obj.params['controls']['pos_v'] = obj.params['controls']['pos_v'] + np.matrix([0,0,button[0]*obj.params['keyboard']['inc_pos_v']]).T





if __name__ == '__main__':
    main()
    
