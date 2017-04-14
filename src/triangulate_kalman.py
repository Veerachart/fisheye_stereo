#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import PointStamped, Vector3Stamped
import message_filters
import tf
import csv
import time
import sys,os
pathname = os.path.dirname(os.path.dirname(sys.argv[0]))
from operator import itemgetter
from decimal import Decimal

if not os.path.isdir(pathname+"/logdata"):
    os.makedirs(pathname+"/logdata")

class Tracker:
    def __init__(self, (t, x, y, z), var=np.identity(3)):
        self.time = t
        self.prev_state = np.array([x,y,z,0,0,0])           # x, y, z, vx, vy, vz
        self.state = np.array([x,y,z,0,0,0])
        self.predicted_pos = None
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0]],np.float64)
        self.P = 1*np.identity(self.state.size)     # Covariance of current state
        self.P[0:3, 0:3] = var                      # for x, y, z
        self.P_prev = self.P
        self.Q = np.zeros((6,6))                    # Covariance of system
        self.Q[0:3, 0:3] = 0.0009*np.identity(3)    # Low position variance
        self.Q[3:, 3:] = 0.0025*np.identity(3)        # High velocity variance
        # self.R = np.identity(self.H.shape[0])       # Covariance of measurement, will be changed
        self.logPrediction = True
        
        U, s, self.rotation = np.linalg.svd(self.P[0:3,0:3])
        self.sd = np.sqrt(s)*3           # 3 Times of SD for checking
        print self.sd
        
    
    def predictConstVel(self):
        now = rospy.Time.now().to_time()
        self.delta_t = now - self.time
        self.time = now
        F = np.array([[1,0,0,self.delta_t,0           ,0           ],
                      [0,1,0,0           ,self.delta_t,0           ],
                      [0,0,1,0           ,0           ,self.delta_t],
                      [0,0,0,1           ,0           ,0           ],
                      [0,0,0,0           ,1           ,0           ],
                      [0,0,0,0           ,0           ,1           ]],np.float64)
        self.state = np.dot(F,self.prev_state)      # update state
        self.P = np.add(np.dot(F,np.dot(self.P_prev,F.T)), self.Q)      # update covariance
        self.prev_state = self.state                # store new value to the previous value
        self.P_prev = self.P                        # store new value to the previous value
        U, s, self.rotation = np.linalg.svd(self.P[0:3,0:3])
        self.sd = np.sqrt(s)*3           # 3 Times of SD for checking
        # print self.sd
        
        
    def updateCorrection(self, (t, x_detect, y_detect, z_detect), R):
        # Check distance from predicted state
        detection = np.array([x_detect, y_detect, z_detect])
        
        x = np.dot(detection-np.dot(self.H,self.state),self.rotation.transpose())
        
        if np.dot(x/self.sd, x/self.sd) < 1:        # detection within the variance of 3SD
            K = np.dot(self.P_prev, np.dot(self.H.transpose(),np.linalg.inv(np.add(np.dot(self.H,np.dot(self.P_prev,self.H.transpose())),R))))
            self.state = np.add(self.prev_state, np.dot(K,np.subtract(detection,np.dot(self.H,self.prev_state))))
            self.P = np.subtract(self.P_prev,np.dot(K,np.dot(self.H,self.P_prev)))
            self.prev_state = self.state                # store new value to the previous value
            self.P_prev = self.P                        # store new value to the previous value
            U, s, self.rotation = np.linalg.svd(self.P[0:3,0:3])
            self.sd = np.sqrt(s)*3           # 3 Times of SD for checking
            print self.sd
        else:
            rospy.loginfo("Outside 3 SD")


coeffs1 = [-0.003125, 0, 0.001029, 0, 0.007671, 0, 0.013237, 0, 1.492357]
coeffs2 = [-0.003934, 0, 0.002062, 0, 0.010611, 0, 0.017052, 0, 1.495884]
R1 = np.array([[0.999601,  0.027689, -0.004448], [-0.027689, 0.999617, 0.000062], [ 0.004448,  0.000062,  0.999990]])
R2 = np.array([[0.997059, -0.075636,  0.012355], [ 0.075567, 0.997123, 0.005980], [-0.012772,  -0.005029, 0.999906]])
mu1 = 157.1979;     mv1 = 157.2336;     u01 = 385.21;       v01 = 385.24;
m1 = (mu1+mv1)/2.
mu2 = 156.4238;     mv2 = 156.4208;     u02 = 385.14;       v02 = 385.32;
m2 = (mu2+mv2)/2.


class Triangulator:
    def __init__(self, baseline = 1):
        self.tracker = None
        self.start = 0
        self.save = True
        if self.save:
            self.file_name = pathname+"/logdata/"+time.strftime("%Y%m%d-%H%M")+"_blimp.csv"
            self.f = open(self.file_name, 'wb')
            self.writer = csv.writer(self.f)
            self.writer.writerow("time,x,y,z,vx,vy,vz,x_det,y_det,z_det,s11,s12,s13,s21,s22,s23,s31,s32,s33".split(','))
#            self.writer.writerow("time,time_r,x,y,z,x_det,y_det,z_det,x_r,y_r,z_r,isTrack,u_left,v_left,u_right,v_right".split(','))
        self.baseline = baseline
        self.p1_sub = message_filters.Subscriber("/cam_left/blimp_center", PointStamped)
        self.p2_sub = message_filters.Subscriber("/cam_right/blimp_center", PointStamped)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.p1_sub, self.p2_sub], 3, 0.04)
        self.sync.registerCallback(self.triangulateCallback)
        self.broadcaster = tf.TransformBroadcaster()
        
        self.sigma_u = 5.
        
        self.detection = None
        self.cov_detect = None
        self.newDetection = False
        
        
    def triangulateCallback(self, p_left, p_right):
        #rospy.loginfo("Synced")
        t = max(p_left.header.stamp.to_time(), p_right.header.stamp.to_time())
        if self.start == 0:
            self.start = t
        if (p_left.point.x == 0 and p_left.point.y == 0) or (p_right.point.x == 0 and p_right.point.y == 0):
#            if self.tracker:
#                if self.save:
#                    try:
#                        data = "%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f" % (t-self.start,t2-self.start, self.tracker.state[0], self.tracker.state[1], self.tracker.state[2], 0, 0, 0, p_left.point.x, p_left.point.y, p_right.point.x, p_right.point.y)
#                        self.writer.writerow(data.split(','))
#                    except csv.Error as e:
#                        sys.exit('File %s, line %d: %s' % (self.file_name, self.writer.line_num, e))
#            else:
#                if self.save:
#                    try:
#                        data = "%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f" % (t-self.start,t2-self.start, 0, 0, 0, 0, 0, 0, p_left.point.x, p_left.point.y, p_right.point.x, p_right.point.y)
#                        self.writer.writerow(data.split(','))
#                    except csv.Error as e:
#                        sys.exit('File %s, line %d: %s' % (self.file_name, self.writer.line_num, e))
            rospy.loginfo("Not detected")
            return
        x = (p_left.point.x-u01)/mu1;
        y = (p_left.point.y-v01)/mv1;
        phi = np.arctan2(y,x);
        r = np.sqrt(x**2 + y**2)
        p = coeffs1[:]
        p.append(-r)
        thetas = np.roots(p)
        for theta in thetas:
            if np.imag(theta) == 0:
                if 0 < np.real(theta) < np.pi/2:
                    theta = np.double(np.real(theta))
                    break
        else:
            print "Unable to find theta", thetas
            return None
        u_cam = np.array([[np.sin(theta)*np.cos(phi)], [np.sin(theta)*np.sin(phi)], [np.cos(theta)]])
        rect_r = np.dot(R1,u_cam)
        psi1 = np.arcsin(rect_r[0,0])
        beta1 = np.arctan2(rect_r[1,0],rect_r[2,0])
        
        J_theta1 = coeffs1[8] + 3*coeffs1[6]*theta**2 + 5*coeffs1[4]*theta**4 + 7*coeffs1[2]*theta**6 + 9*coeffs1[0]*theta**8
        var_theta_phi1 = np.diag([self.sigma_u**2/(m1*J_theta1)**2, self.sigma_u/(m1*r)**2])
        
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_psi = np.cos(psi1)
        s_psi = np.sin(psi1)
        c_beta = np.cos(beta1)
        s_beta = np.sin(beta1)
        
        J_ucam1 = np.array([[c_theta*c_phi, -s_theta*s_phi], [c_theta*s_phi, s_theta*c_phi], [-s_theta, 0]])

        var_ucam1 = np.dot(np.dot(J_ucam1, var_theta_phi1), J_ucam1.transpose())
        
        var_R1 = np.dot(np.dot(R1, var_ucam1), R1.transpose())

        J_beta_psi1 = np.array([[c_psi, -s_psi*s_beta, -s_psi*c_beta], [0, c_beta/(c_psi+1e-6), -s_beta/(c_psi+1e-6)]])

        var_beta_psi1 = np.dot(np.dot(J_beta_psi1, var_R1), J_beta_psi1.transpose())

        x = (p_right.point.x-u02)/mu2;
        y = (p_right.point.y-v02)/mv2;
        phi = np.arctan2(y,x);
        r = np.sqrt(x**2 + y**2)
        p = coeffs2[:]
        p.append(-r)
        thetas = np.roots(p)
        for theta in thetas:
            if np.imag(theta) == 0:
                if 0 < np.real(theta) < np.pi/2:
                    theta = np.double(np.real(theta))
                    break
        else:
            print "Unable to find theta", thetas
            return None
        u_cam = np.array([[np.sin(theta)*np.cos(phi)], [np.sin(theta)*np.sin(phi)], [np.cos(theta)]])
        rect_r = np.dot(R2,u_cam)
        psi2 = np.arcsin(rect_r[0,0])
        beta2 = np.arctan2(rect_r[1,0],rect_r[2,0])
        
        J_theta2 = coeffs2[8] + 3*coeffs2[6]*theta**2 + 5*coeffs2[4]*theta**4 + 7*coeffs2[2]*theta**6 + 9*coeffs2[0]*theta**8
        var_theta_phi2 = np.diag([self.sigma_u**2/(m2*J_theta2)**2, self.sigma_u/(m2*r)**2])
        
        c_theta2 = np.cos(theta)
        s_theta2 = np.sin(theta)
        c_phi2 = np.cos(phi)
        s_phi2 = np.sin(phi)
        c_psi2 = np.cos(psi2)
        s_psi2 = np.sin(psi2)
        c_beta2 = np.cos(beta2)
        s_beta2 = np.sin(beta2)
        
        J_ucam2 = np.array([[c_theta2*c_phi2, -s_theta2*s_phi2], [c_theta2*s_phi2, s_theta2*c_phi2], [-s_theta2, 0]])

        var_ucam2 = np.dot(np.dot(J_ucam2, var_theta_phi2), J_ucam2.transpose())
        
        var_R2 = np.dot(np.dot(R2, var_ucam1), R2.transpose())

        J_beta_psi2 = np.array([[c_psi2, -s_psi2*s_beta2, -s_psi2*c_beta2], [0, c_beta2/(c_psi2+1e-6), -s_beta2/(c_psi2+1e-6)]])

        var_beta_psi2 = np.dot(np.dot(J_beta_psi2, var_R2), J_beta_psi2.transpose())

        disparity = psi1-psi2
        
        if abs(beta1 - beta2) < 0.15:        # On the same epipolar line
            if psi1 > psi2:
                rho = baseline*np.cos(psi2)/np.sin(disparity)
                if rho <= 10:
                    x_out = rho*np.sin(psi1)
                    y_out = rho*np.cos(psi1)*np.sin(beta1)
                    z_out = rho*np.cos(psi1)*np.cos(beta1)
                    
                    var_combi = np.zeros((4,4))
                    var_combi[0:2,0:2] = var_beta_psi1
                    var_combi[2:,2:] = var_beta_psi2
                    
                    J_p = np.zeros((3,4))
                    temp = -baseline*(np.array([[c_psi2*(-s_psi*np.cos(disparity)/(np.sin(disparity)**2) + c_psi/np.sin(disparity))],
                                                [-c_psi2*s_beta*(c_psi*np.cos(disparity)/(np.sin(disparity)**2) + s_psi/np.sin(disparity))],
                                                [-c_psi2*c_beta*(c_psi*np.cos(disparity)/(np.sin(disparity)**2) + s_psi/np.sin(disparity))]]))
                    J_p[0, 0] = temp[0, 0]
                    J_p[1, 0] = temp[1, 0]
                    J_p[2, 0] = temp[2, 0]
                    temp = baseline*c_psi2/np.sin(disparity) * np.array([[0], [c_psi*c_beta], [-c_psi*s_beta]])
                    J_p[0, 1] = temp[0, 0]
                    J_p[1, 1] = temp[1, 0]
                    J_p[2, 1] = temp[2, 0]
                    temp = baseline*(c_psi2*np.cos(disparity)/(np.sin(disparity))**2 - s_psi2/np.sin(disparity)) * np.array([[s_psi], [c_psi*s_beta], [c_psi*c_beta]])
                    J_p[0, 2] = temp[0, 0]
                    J_p[1, 2] = temp[1, 0]
                    J_p[2, 2] = temp[2, 0]

                    var_p = np.dot(np.dot(J_p, var_combi), J_p.transpose())     # Covariance matrix of the measured data
                    
                    #U, s, self.rotation = np.linalg.svd(var_p)
                    #self.sd = np.sqrt(s) * 3.
                    
                    # Limited by the area
                    if 0 < z_out < 3.5 and -1. < x_out < 5. and -5. < y_out < 6.:
                        self.detection = (t, x_out, y_out, z_out)
                        self.cov_detect = var_p
                        self.newDetection = True
                        if not self.tracker:
                            self.tracker = Tracker(self.detection, self.cov_detect)
                            rospy.loginfo("New")
                            return
                        rospy.loginfo("Detected")
                        return
                            
        rospy.loginfo("No match")
        print beta1, beta2
        #if self.tracker:
        #    if self.tracker.updateNotFound() >= 15:
        #        rospy.loginfo("Lost!")
        #        self.tracker = None
            

                
#        if self.tracker:
#            if self.save:
#                try:
#                    data = "%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f" % (t-self.start,t2-self.start, self.tracker.state[0], self.tracker.state[1], self.tracker.state[2], 0, 0, 0, p_left.point.x, p_left.point.y, p_right.point.x, p_right.point.y)
#                    self.writer.writerow(data.split(','))
#                except csv.Error as e:
#                    sys.exit('File %s, line %d: %s' % (self.file_name, self.writer.line_num, e))
#        else:
#            if self.save:
#                try:
#                    data = "%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f" % (t-self.start,t2-self.start, 0, 0, 0, 0, 0, 0,p_left.point.x, p_left.point.y, p_right.point.x, p_right.point.y)
#                    self.writer.writerow(data.split(','))
#                except csv.Error as e:
#                    sys.exit('File %s, line %d: %s' % (self.file_name, self.writer.line_num, e))
    
    
    def update(self):
        t = rospy.Time.now().to_time()
        if self.tracker:
            self.tracker.predictConstVel()
            if self.newDetection:
                self.tracker.updateCorrection(self.detection, self.cov_detect)
            self.broadcaster.sendTransform((self.tracker.state[0],self.tracker.state[1],self.tracker.state[2]),
                                           tf.transformations.quaternion_from_euler(0,0,0),
                                           rospy.Time.now(),
                                           '/blimp',
                                           '/world')
            if self.save:
                try:
                    data = "%.6f,%.4f,%.4f,%.4f,%.2E,%.2E,%.2E,%.4f,%.4f,%.4f,%.6E,%.6E,%.6E,%.6E,%.6E,%.6E,%.6E,%.6E,%.6E" % (t-self.start, self.tracker.state[0], self.tracker.state[1], self.tracker.state[2], Decimal(self.tracker.state[3]), Decimal(self.tracker.state[4]), Decimal(self.tracker.state[5]), self.detection[1], self.detection[2], self.detection[3], Decimal(self.tracker.P[0,0]), Decimal(self.tracker.P[0,1]), Decimal(self.tracker.P[0,2]), Decimal(self.tracker.P[1,0]), Decimal(self.tracker.P[1,1]), Decimal(self.tracker.P[1,2]), Decimal(self.tracker.P[2,0]), Decimal(self.tracker.P[2,1]), Decimal(self.tracker.P[2,2]))
                    self.writer.writerow(data.split(','))
                except csv.Error as e:
                    sys.exit('File %s, line %d: %s' % (self.file_name, self.writer.line_num, e))
            if (self.tracker.sd > 1.5).any():
                rospy.loginfo("Low accuracy, erase")
                self.tracker = None
        self.newDetection = False




if __name__ == '__main__':
    rospy.init_node('triangulate')
    baseline = 3.80
    
    triangulator = Triangulator(baseline)
    r = rospy.Rate(25)
    while not rospy.is_shutdown():
        triangulator.update()
        r.sleep()
    
    rospy.spin()
