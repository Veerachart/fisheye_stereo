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

global start, x_out, y_out, z_out
start = 0
coeffs1 = [-0.001235, 0, 0.001223, 0, -0.007934, 0, 0.017717, 0, 1.512327]
coeffs2 = [-0.001877, 0, 0.010240, 0, -0.033468, 0, 0.045662, 0, 1.499236]
R1 = np.array([[0.998937,  0.035785, -0.030509], [-0.035785,  0.999359,  0.000546], [0.030509,  0.000546,  0.999534]])
R2 = np.array([[0.999744,  0.019123, -0.012076], [-0.019195,  0.999798, -0.005883], [0.011961,  0.006114,  0.999910]])
mu1 = 321.3991;     mv1 = 321.4652;     u01 = 761.98;       v01 = 772.98;
mu2 = 321.3305;     mv2 = 321.2910;     u02 = 760.76;       v02 = 770.55;

if not os.path.isdir(pathname+"/logdata"):
    os.makedirs(pathname+"/logdata")
file_name = pathname+"/logdata/"+time.strftime("%Y%m%d-%H%M")+"_blimp.csv"
#f = open(file_name, 'wb')
#writer = csv.writer(f)
#writer.writerow("time,x,y,z".split(','))
x_out = 0
y_out = 0
z_out = 0

def triangulateCallback(p_left, p_right):
    global start, x_out, y_out, z_out
    if len(p_left.polygon.points) == 0 or len(p_right.polygon.points) == 0:
        return
    t = p_left.header.stamp.to_time()
    if start == 0:
        start = t
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
        psi = np.arcsin(u_cam[0,0])
        beta = np.arctan2(u_cam[1,0],u_cam[2,0])
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
        psi = np.arcsin(u_cam[0,0])
        beta = np.arctan2(u_cam[1,0],u_cam[2,0])
        psi_beta_list_right.append((psi,beta,point.z))
        
    sorted(psi_beta_list_left, key=itemgetter(2), reverse=True)
    sorted(psi_beta_list_right, key=itemgetter(2), reverse=True)
    
    i=0
    j=0
    found = False
    while not found:
        z1=psi_beta_list_left[i][2]
        z2=psi_beta_list_right[j][2]
        if z1 < z2:     # right camera has better membership value
            (psi2,beta2,z2) = psi_beta_list_right[j]
            for (psi1,beta1,z1) in psi_beta_list_left:
                if abs(beta1-beta2) < 0.08:     # On the same epipolar line
                    rho = baseline*np.cos(psi2)/np.sin(psi1-psi2)
                    x_out = rho*np.sin(psi1)
                    y_out = rho*np.cos(psi1)*np.sin(beta1)
                    z_out = rho*np.cos(psi1)*np.cos(beta1)
                    
                    broadcaster.sendTransform((x_out, y_out, z_out),
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
            (psi1,beta1,z1) = psi_beta_list_left[i]
            for (psi2,beta2,z2) in psi_beta_list_right:
                if abs(beta1-beta2) < 0.08:     # On the same epipolar line
                    rho = baseline*np.cos(psi2)/np.sin(psi1-psi2)
                    x_out = rho*np.sin(psi1)
                    y_out = rho*np.cos(psi1)*np.sin(beta1)
                    z_out = rho*np.cos(psi1)*np.cos(beta1)
                    
                    broadcaster.sendTransform((x_out, y_out, z_out),
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
            break
            
#    try:
#        data = "%.9f,%.3f,%.3f,%.3f" % (t-start, x_out, y_out, z_out)
#        writer.writerow(data.split(','))
#    except csv.Error as e:
#        sys.exit('File %s, line %d: %s' % (file_name, writer.line_num, e))




if __name__ == '__main__':
    rospy.init_node('triangulate')
    baseline = 2.1184
    p1_sub = message_filters.Subscriber("/cam_left/blimp_center", PolygonStamped)
    p2_sub = message_filters.Subscriber("/cam_right/blimp_center", PolygonStamped)
    sync = message_filters.ApproximateTimeSynchronizer([p1_sub, p2_sub], 10, 1)
    sync.registerCallback(triangulateCallback)
    broadcaster = tf.TransformBroadcaster()
    
    
    rospy.spin()
