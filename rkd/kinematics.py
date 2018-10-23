#Kinematics
import numpy as np
from numpy import *
from math import *
from rkd.util import *
from rkd.transformations import *
from rkd.mathematical_algorithms import *

def jacobian(*args):
    """
    Calculates the Jacobian matrix to determine the differential kinematics of manipulators / robots
    """
    Ts = []
    type = []
    for k in args:
        Ts.append(htmDH(k[0],k[1],k[2],k[3]))
        if len(k)>4:
            type.append(k[4])
        else:
            type.append('r')
    dof = len(args)

    n = dof
    M_ = np.zeros((6,n))
    for i in range(dof):
        if type[i]=='r':
            jp = np.cross(z(i,Ts), (p(n,Ts) - p(i,Ts)))
            jo = z(i,Ts)
        else:
            jp = z(i,Ts)
            jo = np.zeros((3,1))
        jp = np.row_stack((jp.reshape(3,-1),jo.reshape(3,-1)))
        M_[:,i] = jp.flatten()
    return M_

def z(i,Ts):
    """
    z-dir of every i-Frame wrt 0-Frame
    """
    if i == 0: return np.array([0,0,1])
    MTH = np.eye(4)
    for k in range(i):
        MTH = m_mult(MTH, Ts[k])
    return MTH[:3,2]

def p(i,Ts):
    """
    Position for every i-Frame wrt 0-Frame
    """
    if i == 0: return np.array([0,0,0])
    MTH = np.eye(4)
    for k in range(i):
        MTH = m_mult(MTH,Ts[k])
    return MTH[:3,3]

#Kinematics inverse

def reverse_kinematics(J, b, x0, deg = False,eps = 1e-6):
    """
    Calculates the inverse kinematics from a numerical method

    ** Solved from the numerical method 'Newton - Raphson' **

    *deg* : bool
        ¿Is theta given in degrees?

    IMPORT: You must create your two functions of J (Jacobian), b (initial values ​​evaluated)
    """
    k = 0 #iterations
    while True:
        x = np.linalg.solve(J(x0), -b(x0))
        if norm(x) < eps: break
        x0 += x
        k += 1 #Increase of iterations counter
    if deg:
        return rad2deg(x0), x, k
    return x0, x, k

def j1(x0):
    """
    Calculates the Jacobian matrix evaluated with the initial values
    """
    t1,t2 = x0
    J = np.array([[-100*sin(t1) - 100*sin(t1 + t2), -100*sin(t1 + t2)],
                  [100*cos(t1) + 100*cos(t1 + t2), 100*cos(t1 + t2)]])
    return J

def b1(x0):
    """
    calculates the aproximation of vector 'bi' on the method 'Newton - Rapshon'
    """
    t1,t2 = x0
    F = np.array([100*cos(t1) + 100*cos(t1 + t2) - 100, 100*sin(t1) + 100*sin(t1 + t2) - 100])
    return F

if __name__=="__main__":

    x0 = np.array([pi/6, pi/8])

    print(kinematics_inv(j1, b1, x0, True))