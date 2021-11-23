#!/usr/bin/env python
import rospy
import tf
import time
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from course_agv_nav.srv import Plan,PlanResponse
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt

from my_path_planner import PathPlanner

## TODO import your own planner

class GlobalPlanner:
    def __init__(self):
        self.plan_sx = 0.0
        self.plan_sy = 0.0
        self.plan_gx = 8.0
        self.plan_gy = -8.0
        self.plan_grid_size = 0.2
        self.plan_robot_radius = 0.6
        self.plan_ox = []
        self.plan_oy = []
        self.plan_rx = []
        self.plan_ry = []

        # count to update map
        self.map_count = 0

        self.tf = tf.TransformListener()
        self.goal_sub = rospy.Subscriber('/course_agv/goal',PoseStamped,self.goalCallback)
        self.plan_srv = rospy.Service('/course_agv/global_plan',Plan,self.replan)
        self.path_pub = rospy.Publisher('/course_agv/global_path',Path,queue_size = 1)
        self.map_sub = rospy.Subscriber('/slam_map',OccupancyGrid,self.mapCallback)
        self.maps_sub = rospy.Subscriber('/maps',Float64MultiArray, self.mapCB)

        self.obs_x_pub = rospy.Publisher('/obs_x',Float64MultiArray,queue_size = 10)
        self.obs_y_pub = rospy.Publisher('/obs_y',Float64MultiArray,queue_size = 10)
        self.rate = rospy.Rate(20)
        self.updateMap()
        # self.updateGlobalPose()

        pass
    def mapCB(self, msg):
        self.minx = int(round(min(self.plan_ox)))
        self.miny = int(round(min(self.plan_oy)))
        self.maxx = int(round(max(self.plan_ox)))
        self.maxy = int(round(max(self.plan_oy)))

        self.xwidth = int(round((self.maxx - self.minx) / self.plan_grid_size))
        self.ywidth = int(round((self.maxy - self.miny) / self.plan_grid_size))
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)

        self.obmap = [[False for i in range(self.ywidth)]for i in range(self.xwidth)]
        print(len(msg.data))
        for ix in range(self.xwidth):            
            for iy in range(self.ywidth):
                self.obmap[ix][iy] = msg.data[ix*self.ywidth + iy]
        self.planner.obmap = self.obmap
                
    def goalCallback(self,msg):
        self.plan_goal = msg
        self.plan_gx = msg.pose.position.x
        self.plan_gy = msg.pose.position.y
        #print("get new goal!!! ",self.plan_goal)
        self.replan(0)
        pass

    def collisionCallback(self,msg):
        self.replan(0)
    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        self.plan_sx = self.trans[0]
        self.plan_sy = self.trans[1]

    def replan(self,req):
        print('get request for replan!!!!!!!!')
        start = time.time()
        self.initPlanner()
        #print("init time:", time.time() - start)

        #start = time.time()
        self.updateGlobalPose()
        #print("update time:",time.time() - start)
        
        ## TODO get planner result
        ## e.g. self.plan_rx,self.plan_ry = self.planner.planning(self.plan_sx,self.plan_sy,self.plan_gx,self.plan_gy)
        #start_goal_list = Float64MultiArray(data = {self.plan_sx, self.plan_sy, self.plan_gx, self.plan_gy})
        while self.planner.obmap == None:
            self.rate.sleep()
        #start = time.time()
        self.plan_rx,self.plan_ry = self.planner.planning(self.plan_sx,self.plan_sy,self.plan_gx,self.plan_gy)
        print("replan time:",time.time() - start)

        self.publishPath()
        res = True
        return PlanResponse(res)

    def initPlanner(self):
        map_data = np.array(self.map.data).reshape((-1,self.map.info.height)).transpose()
        ox,oy = np.nonzero(map_data > 50)
        self.plan_ox = (ox*self.map.info.resolution+self.map.info.origin.position.x).tolist()
        self.plan_oy = (oy*self.map.info.resolution+self.map.info.origin.position.y).tolist()
        #print(self.plan_ox)
        #print(self.plan_oy)
        ## TODO init your planner
        ## e.g. self.planner = Planner(...)
        print("planner init...")
        obs_x = Float64MultiArray(data = self.plan_ox)
        obs_y = Float64MultiArray(data = self.plan_oy)
        self.obs_x_pub.publish(obs_x)
        self.obs_y_pub.publish(obs_y)

        self.rate.sleep()
        start = time.time()
        self.planner = PathPlanner(self.plan_ox, self.plan_oy, self.plan_grid_size, self.plan_robot_radius)
        print("INIT time", time.time()-start)

    def mapCallback(self,msg):
        self.map = msg
        pass
    def updateMap(self):
        rospy.wait_for_service('/static_map')
        try:
            getMap = rospy.ServiceProxy('/static_map',GetMap)
            msg = getMap().map
        except:
            e = sys.exc_info()[0]
            #print('Service call failed: %s'%e)
        # Update for planning algorithm
        self.mapCallback(msg)

    def publishPath(self):
        path = Path()
        path.header.seq = 0
        path.header.stamp = rospy.Time(0)
        path.header.frame_id = 'map'
        for i in range(len(self.plan_rx)):
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.plan_rx[len(self.plan_rx)-1-i]
            pose.pose.position.y = self.plan_ry[len(self.plan_rx)-1-i]
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            path.poses.append(pose)
        self.path_pub.publish(path)


def main():
    rospy.init_node('global_planner')
    gp = GlobalPlanner()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
