from sympy import *
from sympy.matrices import Matrix

def rotz(theta, deg=False):
    """
    Calculates the rotation matrix about the z-axis

    *theta* : float, int or symbolic
        Rotation angle (given in radians by default)

    *deg* : bool
        ¿Is theta given in degrees?
    """
    if deg: # If theta is given in degrees -> convert to radians
        theta = theta*pi/180
    ct = cos(theta)
    st = sin(theta)
    R = Matrix([[ct, -st, 0],
                  [st, ct, 0],
                  [0, 0, 1]])
    return R


def roty(theta, deg=False):
    """
    Calculates the rotation matrix about the x-axis

    *theta* : float, int or symbolic
        Rotation angle (given in radians by default)

    *deg* : bool
        ¿Is theta given in degrees?
    """
    if deg: # If theta is given in degrees -> convert to radians
        theta = theta*pi/180
    ct = cos(theta)
    st = sin(theta)
    R = Matrix([[1, 0, 0],
                [0, ct, -st],
                [0, st, ct]])
    return R


def rotx(theta, deg=False):
    """
    Calculates the rotation matrix about the x-axis

    *theta* : float, int or symbolic
        Rotation angle (given in radians by default)

    *deg* : bool
        ¿Is theta given in degrees?
    """
    if deg: # If theta is given in degrees -> convert to radians
        theta = theta*pi/180
    ct = cos(theta)
    st = sin(theta)
    R = Matrix([[1, 0, 0],
                [0, ct, -st],
                [0, st, ct]])
    return R


def dh(a,alpha,d,theta):
    """
    Denavit-Hartenberg matrix
    """
    M = Matrix([[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta)],
                  [sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta)],
                  [0,sin(alpha),cos(alpha),d],
                  [0,0,0,1]])
    return M
    

def htm2eul(H, seq="zxz"):
    if seq in ("ZXZ","zxz"):
        return _htm2zxz(H)
    elif seq in ("ZYZ","zyz"):
        return _htm2zyz(H)
    else:
        pass
        # raise exception (to impl)

def _htm2zxz(H):
    """
    Calculates ZXZ Euler Angles from a homogenous transformation matrix
    """
    R = H[:3,:3] # rotation sub-matrix
    r33,r13,r23,r31,r32,r11,r12,r21 = R[2,2],R[0,2],R[1,2],R[2,0],R[2,1],R[0,0],R[0,1],R[1,0]
    if r23!=0 or r13!=0:
        theta = atan2(sqrt(1-r33**2), r33)
        phi = atan2(r13, -r23)
        psi = atan2(r31, r32)
    elif r33==1:
        theta = 0
        phi = 0
        psi = atan2(r21, r11)
    elif r33==-1:
        theta = pi
        psi = 0
        phi = atan2(-r21, -r11)
    else:
        theta = atan2(sqrt(1-r33**2), r33)
        phi = atan2(r13,-r23)
        psi = atan2(r31,r32)
    return phi,theta,psi




if __name__=="__main__":
    H = Matrix([[0,0,1,0], [0,-1,0,0], [1,0,0,0], [0,0,0,1]])
    phi,th,psi = htm2eul(H)
    print(phi)
