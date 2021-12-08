import math
import numpy as np


M_DIST_TH = 0.6  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

############################################################### 
################## System description ########################
############################################################### 
# x_k = f(x_{k-1}, u_k) + w_k
# y_k = h(x_k) + v_k
# where h() and f() are nonlinear function
# x_{k-1}, x_k are the state vector at timestamp k-1 and k
# w_k and v_k are system noise and observation noise
# y_k is the observation at timestamp k
# Notation: Jf(x) and Jh(x) are the Jacobian matrix of f(x) and h(x) 
#######################################################################

# EKF system noise density: Qx = E[w_k * w_k']
Qx = np.diag([0.35, 0.35, np.deg2rad(15.0)]) ** 2
u_match = 500
u_variance = np.identity(3)
# u_variance = np.array([
#     [1/np.log2(1.00001 + u_match), 0.0, 0.0],
#     [0.0, 1/np.log2(1.00001 + u_match), 0.0],
#     [0.0, 0.0, 0.5/np.log2(1.00001 + u_match)]
#     ])

class EKF():
    def __init__(self):
        pass
    
    # input: posteriori estimated state and its covariance at timestamp k, system input u
    # output: posteriori estimated state and its covariance at timestamp k+1
    def estimate(self, last_xEst, last_PEst, u, z):        
        #Predict

        #motion model: odom_model
        overline_xEst = self.odom_model(last_xEst, u)
        # Jf = (\partial f)/(\partial x_{t-1})
        # V = (\partial f)/(\partial u_{t-1})
        Jf = self.calJf(last_xEst, u)
        V = self.calJf2(last_xEst)
        R = np.dot(np.dot(V, u_variance), np.dot(u_variance.T, V.T))
        overline_PEst = np.dot(np.dot(Jf, last_PEst), Jf.T) + R

        #uncertainty propagation: jacob_motion
        # Jh = (\patial h) / (\patial x_t) = I
        Jh = self.calJh(overline_xEst)
        K = np.dot(np.dot(overline_PEst, Jh.T), np.linalg.inv(np.dot(np.dot(Jh, overline_PEst), Jh.T) + Qx))
        xEst = overline_xEst + np.dot(K, z - overline_xEst)
        PEst = np.dot(np.identity(3) - np.dot(K, Jh), overline_PEst)

        return xEst, PEst

    def calJf(self, x, u):
        # x->[x, y, theta]T
        # u->[dx, dy, dtheta]T
        theta = x[2,0]
        dx=u[0,0]
        dy=u[1,0]
        Jf = np.identity(3)
        Jf[0,2]=-dx*math.sin(theta) - dy*math.cos(theta)
        Jf[1,2]=dx*math.cos(theta) - dy*math.sin(theta)
        
        return Jf

    def calJf2(self, x):
        theta = x[2,0]
        Jf = np.array([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta),0], [0,0,1]])
        return Jf

    def calJh(self, x):
        Jh = np.eye(3)
        return Jh

    # input: posteriori estimated state at timestamp k, system input u
    # output: priori estimated state at timestamp k+1 
    def odom_model(self, x, u):
        """
            x = [x,y,w,...,xi,yi,...]T
            u = [ox,oy,ow]T
        """
        x[0,0] += u[0,0] * math.cos(x[2,0]) - u[1,0] * math.sin(x[2,0])
        x[1,0] += u[0,0] * math.sin(x[2,0]) + u[1,0] * math.cos(x[2,0])
        x[2,0] += u[2,0]

        return x

    def obs_model(self, x):
        pass

    def fuse(self, x, z):
        pass

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
