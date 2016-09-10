#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import PolygonStamped, Point32
import message_filters
import tf
import csv
import time
import sys,os
pathname = os.path.dirname(os.path.dirname(sys.argv[0]))
from operator import itemgetter

if not os.path.isdir(pathname+"/logdata"):
    os.makedirs(pathname+"/logdata")

class Tracker:
    def __init__(self, x=0, y=0, z=0):
        self.prev_pos = (x,y,z)
        self.pos = (x,y,z)
        self.predicted_pos = None
        self.isTracked = False
        self.lengthOfTrack = 0
        self.frameSinceLastTrack = 0
        
    
    def updateTrack(self, detected):
        self.predict()
        dis = self.distance(detected)
            
        # Within range
        if self.isTracked:
            # Being tracked
            self.prev_pos = self.pos        # save old data
            self.pos = detected
            self.isTracked = True
        else:
            self.prev_pos = self.pos        # save old data
            self.pos = detected
            self.lengthOfTrack += 1
            if self.lengthOfTrack >= 5:
                # Get data long enough, set it as being tracked
                self.isTracked = True
    
    
    def updateNotFound(self):
        if self.isTracked:
            # Being tracked, maybe just a miss of data, keep tracking
            self.prev_pos = self.pos        # save old data
            self.pos = self.predicted_pos
            self.frameSinceLastTrack += 1
        else:
            # Not yet long enough to say that this is a good data
            # Should be discarded 
            self.prev_pos = self.pos        # save old data
            self.pos = self.predicted_pos
            self.frameSinceLastTrack = 15
        return self.frameSinceLastTrack
                
    def distance(self, new):
        return (new[0]-self.predicted_pos[0])**2 + (new[1]-self.predicted_pos[1])**2 + (new[2]-self.predicted_pos[2])**2
        
        
    def predict(self):
        self.predicted_pos = (self.pos[0] + (self.pos[0]-self.prev_pos[0])/2., \
                              self.pos[1] + (self.pos[1]-self.prev_pos[1])/2., \
                              self.pos[2] + (self.pos[2]-self.prev_pos[2])/2.)


coeffs1 = [-0.001235, 0, 0.001223, 0, -0.007934, 0, 0.017717, 0, 1.512327]
coeffs2 = [-0.001877, 0, 0.010240, 0, -0.033468, 0, 0.045662, 0, 1.499236]
R1 = np.array([[0.998937,  0.035785, -0.030509], [-0.035785,  0.999359,  0.000546], [0.030509,  0.000546,  0.999534]])
R2 = np.array([[0.999744,  0.019123, -0.012076], [-0.019195,  0.999798, -0.005883], [0.011961,  0.006114,  0.999910]])
mu1 = 321.3991;     mv1 = 321.4652;     u01 = 761.98;       v01 = 772.98;
mu2 = 321.3305;     mv2 = 321.2910;     u02 = 760.76;       v02 = 770.55;


class Triangulator:
    def __init__(self, baseline = 1):
        self.tracker = None
        self.start = 0
        self.file_name = pathname+"/logdata/"+time.strftime("%Y%m%d-%H%M")+"_blimp.csv"
        self.f = open(self.file_name, 'wb')
        self.writer = csv.writer(self.f)
        self.writer.writerow("time,x,y,z,isTrack,f_left,f_right".split(','))
        self.baseline = baseline
        self.p1_sub = message_filters.Subscriber("/cam_left/blimp_center", PolygonStamped)
        self.p2_sub = message_filters.Subscriber("/cam_right/blimp_center", PolygonStamped)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.p1_sub, self.p2_sub], 10, 0.067)
        self.sync.registerCallback(self.triangulateCallback)
        self.broadcaster = tf.TransformBroadcaster()
        
        
    def triangulateCallback(self, p_left, p_right):
        t = p_left.header.stamp.to_time()
        if self.start == 0:
            self.start = t
        if len(p_left.polygon.points) == 0 or len(p_right.polygon.points) == 0:
            if self.tracker:
                if self.tracker.updateNotFound() >= 15:
                    rospy.loginfo("Lost!")
                    self.tracker = None
                    try:
                        data = "%.9f,%.3f,%.3f,%.3f,%d,%.3f,%.3f" % (t-self.start, 0, 0, 0, 0, 0, 0)
                        self.writer.writerow(data.split(','))
                    except csv.Error as e:
                        sys.exit('File %s, line %d: %s' % (self.file_name, self.writer.line_num, e))
                else:
                    try:
                        data = "%.9f,%.3f,%.3f,%.3f,%d,%.3f,%.3f" % (t-self.start, self.tracker.pos[0], self.tracker.pos[1], self.tracker.pos[2], self.tracker.isTracked, 0, 0)
                        self.writer.writerow(data.split(','))
                    except csv.Error as e:
                        sys.exit('File %s, line %d: %s' % (self.file_name, self.writer.line_num, e))
            else:
                try:
                    data = "%.9f,%.3f,%.3f,%.3f,%d,%.3f,%.3f" % (t-self.start, 0, 0, 0, 0, 0, 0)
                    self.writer.writerow(data.split(','))
                except csv.Error as e:
                    sys.exit('File %s, line %d: %s' % (self.file_name, self.writer.line_num, e))
            return
        psi_beta_list_left = []
        psi_beta_list_right= []
        for point in p_left.polygon.points:
            u = point.x;
            v = point.y;
            x = (u-u01)/mu1;
            y = (v-v01)/mv1;
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
                print "Unable to find theta"
                return None
            u_cam = np.array([[np.sin(theta)*np.cos(phi)], [np.sin(theta)*np.sin(phi)], [np.cos(theta)]])
            rect_r = np.dot(R1,u_cam)
            psi = np.arcsin(rect_r[0,0])
            beta = np.arctan2(rect_r[1,0],rect_r[2,0])
            psi_beta_list_left.append((psi,beta,point.z))
            
        for point in p_right.polygon.points:
            u = point.x;
            v = point.y;
            x = (u-u02)/mu2;
            y = (v-v02)/mv2;
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
                print "Unable to find theta"
                return None
            u_cam = np.array([[np.sin(theta)*np.cos(phi)], [np.sin(theta)*np.sin(phi)], [np.cos(theta)]])
            rect_r = np.dot(R1,u_cam)
            psi = np.arcsin(rect_r[0,0])
            beta = np.arctan2(rect_r[1,0],rect_r[2,0])
            psi_beta_list_right.append((psi,beta,point.z))

        psi_beta_list_left = sorted(psi_beta_list_left, key=itemgetter(2), reverse=True)
        psi_beta_list_right = sorted(psi_beta_list_right, key=itemgetter(2), reverse=True)
        
        i=0
        j=0
        found = False
        while not found:
            z1=psi_beta_list_left[i][2]
            z2=psi_beta_list_right[j][2]
            if z1 == 0 and z2 == 0:
                rospy.loginfo("No match")
                if self.tracker:
                    if self.tracker.updateNotFound() >= 15:
                        rospy.loginfo("Lost!")
                        self.tracker = None
                break       # 0 should not be matched with zero, just end
            if z1 < z2:     # right camera has better membership value
                #rospy.loginfo("j = %d, %f", j, z2)
                (psi2,beta2,z2) = psi_beta_list_right[j]
                for (psi1,beta1,z1) in psi_beta_list_left:
                    if abs(beta1-beta2) < 0.06:     # On the same epipolar line
                        if psi1 <= psi2:
                            # In the stereo vision, psi1 must be more than psi2 for the same object
                            continue
                        rho = baseline*np.cos(psi2)/np.sin(psi1-psi2)
                        if rho > 10:
                            # 15 meters away from the camera, that is impossible and would be a misdetection
                            continue
                        x_out = rho*np.sin(psi1)
                        y_out = rho*np.cos(psi1)*np.sin(beta1)
                        z_out = rho*np.cos(psi1)*np.cos(beta1)
                        
                        if not self.tracker:
                            self.tracker = Tracker(x_out, y_out, z_out)
                        else:
                            self.tracker.predict()
                            if self.tracker.distance((x_out, y_out, z_out)) > 0.36:
                                continue
                            self.tracker.updateTrack((x_out, y_out, z_out))
                        
                        self.broadcaster.sendTransform(self.tracker.pos,
                                                        tf.transformations.quaternion_from_euler(0,0,0),
                                                        rospy.Time.now(),
                                                        '/blimp',
                                                        '/world')
                        print z1, z2
                        found = True
                        break
                else:
                    # Arriving here means there is no match, increase index of right camera
                    j += 1
            else:           # left camera has better membership value
                #rospy.loginfo("i = %d, %f", i, z1)
                (psi1,beta1,z1) = psi_beta_list_left[i]
                for (psi2,beta2,z2) in psi_beta_list_right:
                    if abs(beta1-beta2) < 0.06:     # On the same epipolar line
                        if psi1 <= psi2:
                            # In the stereo vision, psi1 must be more than psi2 for the same object
                            continue
                        rho = baseline*np.cos(psi2)/np.sin(psi1-psi2)
                        if rho > 10:
                            # 15 meters away from the camera, that is impossible and would be a misdetection
                            continue
                        x_out = rho*np.sin(psi1)
                        y_out = rho*np.cos(psi1)*np.sin(beta1)
                        z_out = rho*np.cos(psi1)*np.cos(beta1)
                        
                        if not self.tracker:
                            self.tracker = Tracker(x_out, y_out, z_out)
                        else:
                            self.tracker.predict()
                            if self.tracker.distance((x_out, y_out, z_out)) > 0.36:
                                continue
                            self.tracker.updateTrack((x_out, y_out, z_out))
                        
                        self.broadcaster.sendTransform(self.tracker.pos,
                                                        tf.transformations.quaternion_from_euler(0,0,0),
                                                        rospy.Time.now(),
                                                        '/blimp',
                                                        '/world')
                        print z1, z2
                        found = True
                        break
                else:
                    # Arriving here means there is no match, increase index of right camera
                    i += 1
            if i >= len(psi_beta_list_left) or j >= len(psi_beta_list_right):
                rospy.loginfo("No match")
                if self.tracker:
                    if self.tracker.updateNotFound() >= 15:
                        rospy.loginfo("Lost!")
                        self.tracker = None
                break
                
        if self.tracker:
            try:
                data = "%.9f,%.3f,%.3f,%.3f,%d,%.3f,%.3f" % (t-self.start, self.tracker.pos[0], self.tracker.pos[1], self.tracker.pos[2], self.tracker.isTracked, z1, z2)
                self.writer.writerow(data.split(','))
            except csv.Error as e:
                sys.exit('File %s, line %d: %s' % (self.file_name, self.writer.line_num, e))
        else:
            try:
                data = "%.9f,%.3f,%.3f,%.3f,%d,%.3f,%.3f" % (t-self.start, 0, 0, 0, 0, 0, 0)
                self.writer.writerow(data.split(','))
            except csv.Error as e:
                sys.exit('File %s, line %d: %s' % (self.file_name, self.writer.line_num, e))




if __name__ == '__main__':
    rospy.init_node('triangulate')
    baseline = 2.1184
    
    triangulator = Triangulator(baseline)
    
rospy.spin()
