"""
This module has been designed for academic purposes, using SymPy as base library. 
It's easy to check that SymPy is slower than NumPy specially in matrix algebra, 
however SymPy is more convenient to use as didactic tool due to the given facilities 
as the symbolic manipulation, calculation of partial and ordinary derivatives, 
matricial multiplication using asterisk symbol, "init_printing" function and so on.
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sympy import *
from sympy.matrices import Matrix,eye
from rkd.abc import *
    
# ~ ==========================================
# ~ Transformation operations
# ~ ==========================================

def rotz(theta, deg=False):
    """
    Calculates the rotation matrix about the z-axis

    *theta* : float, int or symbolic
        Rotation angle (given in radians by default)

    *deg* : bool
        ¿Is theta given in degrees?
    """
    if deg: # If theta is given in degrees -> convert to radians
        theta = deg2rad(theta)
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
        theta = deg2rad(theta)
    ct = cos(theta)
    st = sin(theta)
    R = Matrix([[ct, 0, st],
                [0, 1, 0],
                [-st, 0, ct]])
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
        theta = deg2rad(theta)
    ct = cos(theta)
    st = sin(theta)
    R = Matrix([[1, 0, 0],
                [0, ct, -st],
                [0, st, ct]])
    return R


def _rot(theta, axis, deg=False):
    if axis in ("X","x",1,"1"):
        R = rotx(theta, deg)
    elif axis in ("Y","y",2,"2"):
        R = roty(theta, deg)
    elif axis in ("Z","z",3,"3"):
        R = rotz(theta, deg)
    else:
        R = eye(3)
    return R
    

def compose_rotations(*rotations):
    """
    Composes rotation matrices -> w.r.t fixed or movable frames
    
    rot: (angle, axis, frame, deg)
    """
    R = eye(3) # I3x3 matrix
    for rot in rotations:
        angle,axis,frame,*_ = rot
        if len(rot)==4:
            deg = rot[-1]
        else:
            deg = False # default value
        crm = _rot(angle,axis)
        if frame in ("world","fixed","global","w","0",0):
            R = crm*R
        elif frame in ("current","movable","local","c","1",1):
            R = R*crm
        else:
            pass # Nothing to do here -> raise except. (to impl.)
            
    return R


def dh(a,alpha,d,theta):
    """
    Return the Denavit-Hartenberg matrix given the four parameters
    (a, alpha, d, theta)
    """
    M = Matrix([[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta)],
                  [sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta)],
                  [0,sin(alpha),cos(alpha),d],
                  [0,0,0,1]])
    return M
    
    
def eul2htm(phi,theta,psi,seq="zxz",deg=False):
    """
    Given a set of Euler Angles (phi,theta,psi) for specific 
    sequence this function return the homogeneous transformation 
    matrix associated. Default sequence is ZXZ.
    """
    if deg: # If angles are given in degrees -> convert to radians
        phi,theta,psi = deg2rad(Matrix([phi,theta,psi])).evalf()
    if seq in ("ZXZ","zxz"):
        H = htmrot(phi,"z")*htmrot(theta,"x")*htmrot(psi,"z")
    else:
        H = eye(4)
    return H
    
    
def htm2eul(H, seq="zxz", deg=False):
    """
    Given a homogeneous transformation matrix this function 
    return the equivalent set of Euler Angles. 
    
    If "deg" is True then Euler Angles are converted to degrees.
    """
    if seq in ("ZXZ","zxz"):
        return _htm2zxz(H, deg)
    elif seq in ("ZYZ","zyz"):
        return _htm2zyz(H, deg)
    else:
        pass # raise exception (to impl)


def _htm2zxz(H, deg=False):
    """
    Calculates ZXZ Euler Angles from a homogeneous transformation matrix
    """
    R = H[:3,:3] # rotation sub-matrix
    r33,r13,r23,r31,r32,r11,r12,r21 = R[2,2],R[0,2],R[1,2],R[2,0],R[2,1],R[0,0],R[0,1],R[1,0]
    if abs(r33) != 1:
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
        
    if deg:
        return rad2deg(phi), rad2deg(theta), rad2deg(psi)
        
    return phi,theta,psi


def htmtra(d):
    """
    Calculate the homogeneous transformation matrix of a translation
    """
    dx,dy,dz = d[0],d[1],d[2]
    M = Matrix([[1,0,0,dx],
                [0,1,0,dy],
                [0,0,1,dz],
                [0,0,0,1]])
    return M
    

def htmrot(theta, axis="z", deg=False):
    """
    Return a homogeneous transformation matrix that represents a 
    rotation "theta" about "axis". 
    """
    if deg: # Is theta given in degrees? -> then convert to radians
        theta = deg2rad(theta)
        
    if axis in ("z","Z",3,"3"):
        R = rotz(theta)
    elif axis in ("y","Y",2,"2"):
        R = roty(theta)
    elif axis in ("x","X",1,"1"):
        R = rotx(theta)
    else:
        R = eye(3) # raise except (to impl)
    H = _rot2htm(R)
    return H


def _rot2htm(R):
    """
    Given a SO(3) matrix return a SE(3) homogeneous 
    transformation matrix.
    """
    _H = R.row_join(zeros(3,1))
    H = _H.col_join(Matrix([0,0,0,1]).T)
    return H
    

def rot2axa(R, deg=False):
    r32,r23 = R[2,1],R[1,2]
    r13,r31 = R[0,2],R[2,0]
    r21,r12 = R[1,0],R[0,1]
    theta = acos((R.trace() - 1)/2)
    k = ( (1/(2*sin(theta)))*Matrix([r32-r23, r13-r31, r21-r12]) ).evalf()
    if deg:
        theta = rad2deg(theta)
    return k,theta
    
def axa2rot(k,theta):
    if isinstance(k,(list,tuple)):
        k = Matrix(k)
    ct = cos(theta)
    st = sin(theta)
    vt = 1 - cos(theta)
    kx,ky,kz = k.normalized()
    r11 = kx**2*vt + ct
    r21 = kx*ky*vt + kz*st
    r31 = kx*kz*vt - ky*st
    r12 = kx*ky*vt - kz*st
    r22 = ky**2*vt + ct
    r32 = ky*kz*vt + kx*st
    r13 = kx*kz*vt + ky*st 
    r23 = ky*kz*vt - kx*st 
    r33 = kz**2*vt + ct 
    R = Matrix([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])
    return R
    
    
def skew(u):
    """
    Return skew-symmetric matrix associated to u vector
    """
    ux,uy,uz = u
    S = Matrix([[0, -wz, wy],
                [wz, 0, -wx], 
                [-wy, wx, 0]])
    return S

    

if __name__=="__main__":
    H = Matrix([[0,0,1,0], [0,-1,0,0], [1,0,0,0], [0,0,0,1]])
    test_robot()
    # ~ test_rb2()
