#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray,Marker
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2
from icp import ICP
from ekf import EKF,STATE_SIZE
import time

MAX_LASER_RANGE = 30

class SLAM_EKF():
    def __init__(self):
        # ros param
        self.robot_x = rospy.get_param('/slam/robot_x',0)
        self.robot_y = rospy.get_param('/slam/robot_y',0)
        self.robot_theta = rospy.get_param('/slam/robot_theta',0)

        self.icp = ICP()
        self.ekf = EKF()

        # odom robot init states
        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]
        self.isFirstScan = True
        self.src_pc = []
        self.tar_pc = []
        self.map = []

        # State Vector [x y yaw]
        self.xOdom = np.zeros((3, 1))
        self.xEst = np.zeros((3, 1))
        self.PEst = np.zeros((3, 3))
        
        # ros topic
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.laserCallback)
        self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)
        self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.landMark_pub = rospy.Publisher('/landMarks',MarkerArray,queue_size=1)
        #self.icp_sub = rospy.Subscriber('u',Float64MultiArray,self.icpCallBack)
        self.tf = tf.TransformListener()

    def icpCallBack(self,msg):
        u = np.zeros((3,1))
        u[0,0] = msg.data[0]
        u[1,0] = msg.data[1]
        u[2,0] = msg.data[2]
        z = self.get_real_loc()        
        self.xEst,self.PEst = self.ekf.estimate(self.xEst,self.PEst,u,z)
        self.publishResult()

    def laserCallback(self,msg):
        print('')
        print('------seq:  ',msg.header.seq)
        np_msg = self.laserToNumpy(msg)
        
        start = time.time()
        z = self.get_real_loc()
        print("get real loc:", time.time() - start)
        
        start = time.time()
        u = self.calc_odometry(np_msg)
        print("ICP done: ", time.time() - start)
        
        start = time.time()
        self.xEst,self.PEst = self.ekf.estimate(self.xEst,self.PEst,u,z)
        print("EKF done: ", time.time() - start)
        
        self.publishResult()

    # rosrun rqt_tf_tree rqt_tf_tree
    def get_real_loc(self):
        #self.tf = tf.TransformListener()
        self.tf.waitForTransform("/base_footprint", "/map", rospy.Time(),
                                    rospy.Duration(2.0))
        l2m, rot = self.tf.lookupTransform('/base_footprint', '/map', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        roll, pitch, yaw_l2m = euler[0], euler[1], euler[2]

        self.tf.waitForTransform("/map", "/world_base", rospy.Time(),
                                    rospy.Duration(4.0))
        m2w, rot = self.tf.lookupTransform('/map', '/world_base', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        roll, pitch, yaw_m2w = euler[0], euler[1], euler[2]
        dx = -l2m[0] * math.cos(yaw_l2m) - l2m[1] * math.sin(yaw_l2m) - m2w[0]
        dy = l2m[0] * math.sin(yaw_l2m) - l2m[1] * math.cos(yaw_l2m) - m2w[1]
        dthela = -yaw_l2m
        z_world = np.array([    [dx * math.cos(yaw_m2w) + dy * math.sin(yaw_m2w)],
                                [- dx * math.sin(yaw_m2w) + dy * math.cos(yaw_m2w)],
                                [dthela - yaw_m2w]
                                ])
        return z_world


    def calc_odometry(self,np_msg):
        if self.isFirstScan:
            self.tar_pc = np_msg
            self.isFirstScan = False
            return np.array([[0.0,0.0,0.0]]).T
        self.src_pc = np_msg
        transform_acc = self.icp.process(self.tar_pc,self.src_pc)
        self.tar_pc = np_msg
        u = self.T2u(transform_acc)
        #cal pure icp odom
        # self.xOdom[0,0] += u[0,0]*math.cos(self.xOdom[2,0]) - u[1,0]*math.sin(self.xOdom[2,0])
        # self.xOdom[1,0] += u[0,0]*math.sin(self.xOdom[2,0]) + u[1,0]*math.cos(self.xOdom[2,0])
        # self.xOdom[2,0] += u[2,0]
        # print('u:',u.T)
        # print("self.xOdom:", self.xOdom.T)
        return u

    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        range_l[range_l == np.inf] = MAX_LASER_RANGE
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc

    def T2u(self,t):
        dw = math.atan2(t[1,0],t[0,0])
        u = np.array([[t[0,2],t[1,2],dw]])
        return u.T
    
    def u2T(self,u):
        w = u[2]
        dx = u[0]
        dy = u[1]

        return np.array([
            [ math.cos(w),-math.sin(w), dx],
            [ math.sin(w), math.cos(w), dy],
            [0,0,1]
        ])

    def publishResult(self):
        # tf
        s = self.xEst.reshape(-1)
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        self.odom_broadcaster.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"ekf_location","world_base")
        # odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world_base"

        odom.pose.pose.position.x = s[0]
        odom.pose.pose.position.y = s[1]
        odom.pose.pose.position.z = 0.001
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.location_pub.publish(odom)

        # s = self.xOdom
        # q = tf.transformations.quaternion_from_euler(0,0,s[2])
        # # odom
        # odom = Odometry()
        # odom.header.stamp = rospy.Time.now()
        # odom.header.frame_id = "world_base"

        # odom.pose.pose.position.x = s[0]
        # odom.pose.pose.position.y = s[1]
        # odom.pose.pose.position.z = 0.001
        # odom.pose.pose.orientation.x = q[0]
        # odom.pose.pose.orientation.y = q[1]
        # odom.pose.pose.orientation.z = q[2]
        # odom.pose.pose.orientation.w = q[3]

        # self.odom_pub.publish(odom)

def main():
    rospy.init_node('slam_node')
    s = SLAM_EKF()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
    # test()
