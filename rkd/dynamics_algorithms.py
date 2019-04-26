from sympy import *
from sympy.matrices import Matrix, eye

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

def T(matrix):
    rows = len(matrix)
    cols = len(matrix[0])

    t = [[0 for x in range(rows)] for y in range(cols)]

    for i in range(rows):
        for j in range(cols):
            t[j][i] = matrix[i][j]
    return t