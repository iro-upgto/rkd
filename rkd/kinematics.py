#Kinematics
import numpy as np
from math import *
from rkd.abc import *
from rkd.util import *
from rkd.transformations import *
from rkd.mathematical_algorithms import *

def jacobiano(*args):
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
    if i == 0: return np.array([0,0,1])
    MTH = np.eye(4)
    for k in range(i):
        MTH = m_mult(MTH, Ts[k])
    return MTH[:3,2]

def p(i,Ts):
    if i == 0: return np.array([0,0,0])
    MTH = np.eye(4)
    for k in range(i):
        MTH = m_mult(MTH,Ts[k])
    return MTH[:3,3]

if __name__=="__main__":
    
    print(jacobiano((100,0,0,pi/6,'r'), (100,0,0,pi/4,'r'), (20,0,0,-pi/6,'r')))