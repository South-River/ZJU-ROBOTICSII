#!/usr/bin/env python

import heapq
from numpy.testing._private.utils import break_cycles
from scipy.spatial import cKDTree
from scipy.spatial import KDTree
import rospy
import numpy as np
import math
import time

# structure of the nearest neighbor


class NeighBor:
    def __init__(self):
        self.distances = []
        self.src_indices = []
        self.tar_indices = []


def accumulate(inputs):
    itr = iter(inputs)
    prev = next(itr)
    for cur in itr:
        yield prev
        prev = prev + cur
    yield prev


class ICP:
    def __init__(self):
        # max iterations
        self.max_iter = rospy.get_param('/icp/max_iter', 10)
        # distance threshold for filter the matching points
        self.dis_th = rospy.get_param('/icp/dis_th', 3)
        # tolerance to stop icp
        self.tolerance = rospy.get_param('/icp/tolerance')
        # min match
        self.min_match = rospy.get_param('/icp/min_match', 2)

        self.corner_num = rospy.get_param('/icp/cornor_num', 150)
        self.past_frame_num = rospy.get_param('/icp/past_frame_num', 0)
        self.past_pc = np.zeros((self.past_frame_num*self.corner_num, 3))

    # ICP process function
    # Waiting for Implementation
    # return: T = (R, t), where T is 2*3, R is 2*2 and t is 2*1
    def process(self, tar_pc, src_pc):
        #print("process called...")

        process_time = time.time()
        #preprocess data        
        src_pc_t = np.transpose(src_pc)
        tar_pc_t = np.transpose(tar_pc)
        [row_src_pc_t, col_src_pc_t] = np.shape(src_pc_t)
        [row_tar_pc_t, col_tar_pc_t] = np.shape(tar_pc_t)
        src_pc = np.transpose(self.linearInter(src_pc_t, row_src_pc_t))
        tar_pc = np.transpose(self.linearInter(tar_pc_t, row_tar_pc_t))
        
        # get corner points
        corner_src_pc_t = self.getCornerPoint(np.transpose(src_pc),100)
        corner_tar_pc_t = self.getCornerPoint(np.transpose(tar_pc),self.corner_num)
        src_pc = np.transpose(corner_src_pc_t)
        tar_pc = np.transpose(corner_tar_pc_t)

        # # add past data
        # tar_pc_new = np.zeros((3, (self.past_frame_num+1)*self.corner_num))
        # tar_pc_new[:, 0:self.corner_num] = tar_pc[:, :]
        # tar_pc_new[:, self.corner_num: (self.past_frame_num+1) * self.corner_num] = np.transpose(self.past_pc[:, :])
        # tar_pc = tar_pc_new

        Transform_acc = np.eye(3)
        prev_error = 0
        findNearest_timecost = 0
        getTransform_timecost = 0
        for i in range(self.max_iter):

            p = time.time()
            neigh_ = self.findNearest(
                np.transpose(src_pc), np.transpose(tar_pc))
            findNearest_timecost += time.time() - p

            src_pc_xy = np.ones((2, len(neigh_.src_indices)))
            tar_chorder = np.ones((2, len(neigh_.src_indices)))

            for j in range(len(neigh_.src_indices)):
                src_pc_xy[0:2, j] = src_pc[0:2, neigh_.src_indices[j]]
                tar_chorder[0:2, j] = tar_pc[0:2, neigh_.tar_indices[j]]

            p = time.time()
            Transform = self.getTransform(np.transpose(src_pc_xy), np.transpose(tar_chorder))
            getTransform_timecost += time.time()-p

            Transform_acc = np.dot(Transform, Transform_acc)
            src_pc = np.dot(Transform, src_pc)

            mean_error = 0
            for j in range(len(neigh_.distances)):
                mean_error += neigh_.distances[j]
            mean_error /= len(neigh_.distances)

            #print("mean error:", mean_error)
            if abs(prev_error - mean_error) < self.tolerance:
                print("jump out")
                break
            prev_error = mean_error

        print("Transform_acc:")
        print(Transform_acc)
        print("findNearest cost:", findNearest_timecost)
        print("getTransform cost:", getTransform_timecost)
        print("process cost: ", time.time()-process_time)

        self.past_pc[:, :] = np.transpose(tar_pc[:, 0:self.past_frame_num*self.corner_num])
        # for i in range(self.past_frame_num*self.corner_num):
        #     print("past pc:",i,self.past_pc[i])

        return Transform_acc

    # Linear interpolation of NaN data
    def linearInter(self, list, list_len):
        NaN_start_idx = 0
        NaN_end_idx = 0
        for i in range(list_len):
            if NaN_end_idx > i:
                i = NaN_end_idx
            if i >= list_len:
                break
            if math.isnan(list[i][0]):
                NaN_start_idx = i
                for j in range(NaN_start_idx+1, list_len):
                    if not math.isnan(list[j][0]):
                        NaN_end_idx = j
                        for k in range(NaN_start_idx, NaN_end_idx):
                            list[k, :] = (k-NaN_start_idx+1)*(list[NaN_end_idx, :] - list[NaN_start_idx-1, :]) / (NaN_end_idx-NaN_start_idx+1)
                        break
                    if j == list_len - 1 and math.isnan(list[j][0]):
                        NaN_end_idx = j+1
                        for k in range(NaN_start_idx, NaN_end_idx):
                            list[k, :] = list[NaN_start_idx-1, :]
                        break
        return list

    def isTriangle(self, a, b, c):
        if a+b <= c or b+c <= a or c+a <= b:
            return False
        else:
            return True

    def calF(self, list, idx):
        N = 6
        [row_list, col_list] = np.shape(list)

        sum_x = 0
        sum_y = 0
        for i in range(N+1):
            sum_x += list[idx - i][0]
            sum_y += list[idx-i][1]
        prev_mean_x = sum_x/(N+1)
        prev_mean_y = sum_y/(N+1)

        sum_x = 0
        sum_y = 0
        for i in range(N+1):
            sum_x += list[(idx+i) % row_list][0]
            sum_y += list[(idx+i) % row_list][1]
        next_mean_x = sum_x/(N+1)
        next_mean_y = sum_y/(N+1)

        l_prev = math.hypot(list[idx][0] - prev_mean_x,
                            list[idx][1] - prev_mean_y)
        l_next = math.hypot(list[idx][0]-next_mean_x, list[idx][1]-next_mean_y)
        l_prev_next = math.hypot(
            prev_mean_x-next_mean_x, prev_mean_y-next_mean_y)

        if not self.isTriangle(l_prev, l_next, l_prev_next):
            return 0

        l = (l_next+l_prev+l_prev_next)/2

        f = math.sqrt((l*(l-l_prev)*(l-l_next)*(l-l_prev_next))) / (l_prev*l_next)
        return f

    def getCornerPoint(self, list, list_len = 50):
        [row_list, col_list] = np.shape(list)
        f_list = np.zeros((row_list, 2))
        corner_list = np.zeros((list_len, 3))
        for i in range(row_list):
            f_list[i][0] = self.calF(list, i)
            f_list[i][1] = int(i)

        largest = heapq.nlargest(list_len, f_list, lambda x: x[0])
        for i in range(list_len):
            corner_list[i, :] = list[int(largest[i][1]), :]

        return corner_list

    # find the nearest points & filter
    # return: neighbors of src and tar
    def findNearest(self, src, tar):
        #print("findNearest called...")
        neigh = NeighBor()
        
        # [row_src, col_src]=np.shape(src)
        # [row_tar, col_tar]=np.shape(tar)
        # for ii in range(row_src):
        #     vec_src = np.transpose(src[ii,0:2])
        #     min = 100
        #     index = 0
        #     dist_temp = 0
        #     for jj in range(row_tar):
        #         vec_tar = np.transpose(tar[jj, 0:2])
        #         dist_temp = self.calcDist(vec_src, vec_tar)
        #         if dist_temp < min:
        #             min = dist_temp
        #             index = jj

        #     if min < self.dis_th:
        #         neigh.distances.append(min)
        #         neigh.src_indices.append(ii)
        #         neigh.tar_indices.append(index)
     
        tree = cKDTree(tar)
        ctree = KDTree(tar)
        for i in range(np.shape(src)[0]):
            dist_temp, index = tree.query(src[i])
            if dist_temp < self.dis_th:
                neigh.distances.append(dist_temp)
                neigh.src_indices.append(i)
                neigh.tar_indices.append(index)

        return neigh

    # Waiting for Implementation
    # return: T = (R, t), where T is 2*3, R is 2*2 and t is 2*1
    def getTransform(self, src, tar):
        #print("getTransform called...")
        T = np.eye(3)
        centroid_src = np.array([[0.0], [0.0]])
        centroid_tar = np.array([[0.0], [0.0]])
        src_ = src
        tar_ = tar
        [row_src, col_src] = np.shape(src)
        num = row_src

        for i in range(num):
            centroid_src[0:2, 0] += np.transpose(src[i, 0:2])
            centroid_tar[0:2, 0] += np.transpose(tar[i, 0:2])
        centroid_src[0:2, 0] /= num
        centroid_tar[0:2, 0] /= num

        for i in range(num):
            src_[i, 0:2] = src[i, 0:2] - np.transpose(centroid_src)
            tar_[i, 0:2] = tar[i, 0:2] - np.transpose(centroid_tar)

        H = np.dot(np.transpose(src_), tar_)

        U, S, V = np.linalg.svd(H)
        Vt = np.transpose(V)
        R = np.dot(np.transpose(Vt), np.transpose(U))
        t = centroid_tar - np.dot(R, centroid_src)
        T[0:2, 0:2] = R
        T[0:2, 2] = t[0:2, 0]
        return T

    def calcDist(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx, dy)