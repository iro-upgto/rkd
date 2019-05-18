import sympy
import numpy as np
from sympy import *
from sympy.matrices import Matrix, eye
from rkd.abc import *

def Dh(a,alpha,d,theta):
    """
    Calculates Denavit-Hartenberg matrix given the four parameters.

    Parameters
    ----------
    a : int, float or symbol
        DH parameter
    alpha : int, float or symbol
        DH parrameter
    d : int, float or symbol
        DH parameter
    theta : int, float or symbol
        DH parameter

    Returns
    -------
    H : :class:`sympy.matrices.dense.MutableDenseMatrix`
        Denavit-Hartenberg matrix (4x4)

    Examples
    --------
    >>> dh(100,pi/2,50,pi/2)
    ⎡0  0  1   0 ⎤
    ⎢            ⎥
    ⎢1  0  0  100⎥
    ⎢            ⎥
    ⎢0  1  0  50 ⎥
    ⎢            ⎥
    ⎣0  0  0   1 ⎦
    """
    H = Matrix([[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta)],
                  [sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta)],
                  [0,sin(alpha),cos(alpha),d],
                  [0,0,0,1]])
    return H

def z(i,Ts):
    """
    z-dir of every i-Frame wrt 0-Frame
    """
    if i == 0: return Matrix([[0],[0],[1]])
    MTH = eye(4)
    for k in range(i):
        MTH = MTH*Ts[k]
    return MTH[:3,2]

def p(i,Ts):
    """
    Position for every i-Frame wrt 0-Frame
    """
    if i == 0: return Matrix([[0],[0],[0]])
    MTH = eye(4)
    for k in range(i):
        MTH = MTH*Ts[k]
    return MTH[:3,3]

def jac(*args):
    """
    Calculates the Jacobian matrix to determine the differential kinematics of manipulators / robots
    """
    Ts = []
    type = []
    for k in args:
        Ts.append(Dh(k[0],k[1],k[2],k[3]))
        if len(k)>4:
            type.append(k[4])
        else:
            type.append('r')
    dof = len(args)

    n = dof
    M_ = zeros(6,n)
    for i in range(dof):
        if type[i]=='r':
            jp = z(i,Ts).cross(p(n,Ts) - p(i,Ts))
            jo = z(i,Ts)
        else:
            jp = z(i,Ts)
            jo = zeros(3,1)
        jp = jp.col_join(jo)
        M_[:,i] = jp
    return M_

def DH(*args):
    Ts = []
    H = eye(4)
    for k in args:
        Ts.append(Dh(k[0], k[1], k[2], k[3]))
    for i in range(len(args)):
        H = H*Ts[i]

    return H

def potential_energy(RT, size_linear_velocity, size_table2, direction_of_gravity = 'y'):
    m, j = [], []
    for i in range(size_table2):
        m.append(eval('m'+str(int("".join(str(i+1))))))
        j.append(eval('J'+str(int("".join(str(i+1))))))
    
    u, U, H = [], [], []
    for i in range(len(RT)):
            H.append(DH(*RT[i]))
    for i in range(len(RT)):
        if direction_of_gravity == 'x' or direction_of_gravity == 'X':
            u.append(H[i][:1,3])
        elif direction_of_gravity == 'y' or direction_of_gravity == 'Y':
            u.append(H[i][1:2,3])
        elif direction_of_gravity == 'z' or direction_of_gravity == 'Z':
            u.append(H[i][2:3,3])
    for i in range(size_linear_velocity):
        U.append((m[i]*g)*u[i])
    return simplify(np.sum(U[:], axis = 0))

def check_lists(p1, p2, p3, p4, p5, p6):
    R = []
    for r in [p1, p2, p3, p4, p5, p6]:
        if r != []:
            R.append(r)
    return R

def sorter_of_the_dynamic_modeling(tau_n, dof):
    q, qp, qpp = [], [], []
    for i in range(dof):
        q.append(eval('q'+str(int("".join(str(i+1))))))
        qp.append(eval('q'+str(int("".join(str(i+1))))+'p'))
        qpp.append(eval('q'+str(int("".join(str(i+1))))+'p'+'p'))

    #*******************************************************
    #           OBTAINING THE "INERTIA MATRIX"
    #*******************************************************
    q_pp = []
    for i in range(dof):
        qpp1, qpp2, qpp3, qpp4, qpp5, qpp6 = [], [], [], [], [], []
        for sumando in tau_n[i][0].args:
            for fc in sumando.args:
                if q1pp == fc:
                    qpp1.append(sumando)
                elif q2pp == fc:
                    qpp2.append(sumando)
                elif q3pp == fc:
                    qpp3.append(sumando)
                elif q4pp == fc:
                    qpp4.append(sumando)
                elif q5pp == fc:
                    qpp5.append(sumando)
                elif q6pp == fc:
                    qpp6.append(sumando)
        for r in [qpp1, qpp2, qpp3, qpp4, qpp5, qpp6]:
            if r != []:
                q_pp.append(r)
    
    r = []
    for i in range(len(q_pp)):
        r.append([np.sum(q_pp[i][:], axis = 0)])
    
    i, res = 0, []
    while i < len(r):
        for j in range(dof):
            res.append(Matrix([simplify(r[i][0]/qpp[j])]))
            i += 1
    
    MH, k = ones(dof, dof), 0
    while k < len(r):
        for i in range(dof):
            for j in range(dof):
                MH[i,j] = simplify(MH[i,j]*res[k])
                k += 1

    return MH