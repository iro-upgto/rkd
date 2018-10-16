#Kinematics
import numpy as np
from math import *
from rkd.abc import *
from rkd.util import *
from rkd.transformations import *

def jacobiano(*args):
    Ts = []
    type = []
    for k in args:
        Ts.append(htmDH(k[0],k[1],k[2],k[4]))
        if len(k)>4:
            type.append(k[4])
        else:
            type.append('r')
    dof = len(args)

    def z(i):
        if i == 0: return np.array([[0],[0],[1]])
        MTH = eye(4)
        for q in range(i):
            MTH = MTH*Ts[q]
        return MTH[:3,2]

    def p(i):
        if i == 0: return np.array([[0],[0],[0]])
        MTH = eye(4)
        for q in range(i):
            MTH = MTH*Ts[q]
        return MTH[:3,3]

    def p_cross(v1, v2):
        f1,_ = v1.shape
        f2,_ = v2.shape

        if f1 == f2:
            r1 = v1[0,0]
            r2 = v1[0,1]
            r3 = v1[0,2]
            t1 = v2[0,0]
            t2 = v2[0,1]
            t3 = v2[0,2]

            i = ((r2*t3)-(r3*t2))
            j = -((r1*t3)-(r3*t1))
            k = ((r1*t2)-(r2*t1))

        return np.array([[i],[j],[k]])

    n = dof
    M = zeros(6,n)
    for i in range(dof):
        if type[i]=='r':
            o = n - i
            jp = p_cross(z(i),o)
            jo = z(i)
        else:
            jp = z(i)
            jo = zeros(3,1)
        jp = jp.col_join(jo)
        M_[:,i] = jp
    return M_


if __name__=="__main__": pass

    print("RESULTADO: ", jacobiano((0,3.1416/2,10,pi/2), (0,3.1416/2, 20, -3.1416/2), (100,0,0,3.1416/6))