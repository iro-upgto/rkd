# Transformations
from numpy import *
import numpy as np
from rkd.util import *
from rkd.abc import *
from rkd.mathematical_algorithms import *

def rot(phi,theta,psi, sec, deg):
    if ((sec == 'XYX') or (sec == 'xyx')):
        H = m_mult(rotx(phi,deg), roty(theta,deg), rotx(psi,deg))
    if ((sec == 'XZX') or (sec == 'xzx')):
        H = m_mult(rotx(phi,deg), rotz(theta,deg), rotx(psi,deg))
    if ((sec == 'YXY') or (sec == 'yxy')):
        H = m_mult(roty(phi,deg), rotx(theta,deg), roty(psi,deg))
    if ((sec == 'YZY') or (sec == 'yzy')):
        H = m_mult(roty(phi,deg), rotz(theta,deg), roty(psi,deg))
    if ((sec == 'ZXZ') or (sec == 'zxz')):
        H = m_mult(rotz(phi,deg), rotx(theta,deg), rotz(psi,deg))
    if ((sec == 'ZYZ') or (sec == 'zyz')):
        H = m_mult(rotz(phi,deg), roty(theta,deg), rotz(psi,deg))
    if ((sec == 'XYZ') or (sec == 'xyz')):
        H = m_mult(rotx(phi,deg), roty(theta,deg), rotz(psi,deg))
    if ((sec == 'XZY') or (sec == 'xzy')):
        H = m_mult(rotx(phi,deg), rotz(theta,deg), roty(psi,deg))
    if ((sec == 'YXZ') or (sec == 'yxz')):
        H = m_mult(roty(phi,deg), rotx(theta,deg), rotz(psi,deg))
    if ((sec == 'YZX') or (sec == 'yzx')):
        H = m_mult(roty(phi,deg), rotz(theta,deg), rotx(psi,deg))
    if ((sec == 'ZXY') or (sec == 'zxy')):
        H = m_mult(rotz(phi,deg), rotx(theta,deg), roty(psi,deg))
    if ((sec == 'ZYX') or (sec == 'zyx')):
        H = m_mult(rotz(phi,deg), roty(theta,deg), rotx(psi,deg))

    return H

def rotz(theta, deg=False):
    """
    Calculates the rotation matrix about the z-axis

    *theta* : float or int
        Rotation angle (given in radians by default)

    *deg* : bool
        ¿Is theta given in degrees?
    """
    if deg: # If theta is given in degrees -> convert to radians
        theta = deg2rad(theta)
    ct = np.cos(theta)
    st = np.sin(theta)
    R = np.array([[ct, -st, 0],
                  [st, ct, 0],
                  [0, 0, 1]])
    return R

def rotx(theta, deg=False):
    """
    Calculates the rotation matrix about the x-axis

    *theta* : float or int
        Rotation angle (given in radians by default)

    *deg* : bool
        ¿Is theta given in degrees?
    """
    if deg: # If theta is given in degrees -> convert to radians
        theta = deg2rad(theta)
    ct = np.cos(theta)
    st = np.sin(theta)
    R = np.array([[1,0,0],
                  [0,ct,-st],
                  [0,st,ct]])
    return R

def roty(theta, deg=False):
    """
    Calculates the rotation matrix about the y-axis

    *theta* : float or int
        Rotation angle (given in radians by default)

    *deg* : bool
        ¿Is theta given in degrees?
    """
    if deg: # If theta is given in degrees -> convert to radians
        theta = deg2rad(theta)
    ct = np.cos(theta)
    st = np.sin(theta)
    R = np.array([[ct,0,st],
                  [0,1,0],
                  [-st,0,ct]])
    return R

def rot2eul(R, axis, deg = False, sol = False):
    """
    Calculates the Euler angles from a rotation matrix with differents combinations of axis

    ** The angles must be given in radians by default **

    Important: The rotation matrix must be 3x3
    """
    
    r11 = R[0,0] # Position in the matrix [1,1]
    r12 = R[0,1] # Position in the matrix [1,2]
    r13 = R[0,2] # Position in the matrix [1,3]
    r21 = R[1,0] # Position in the matrix [2,1]
    r22 = R[1,1] # Position in the matrix [2,2]
    r23 = R[1,2] # Position in the matrix [2,3]
    r31 = R[2,0] # Position in the matrix [3,1]
    r32 = R[2,1] # Position in the matrix [3,2]
    r33 = R[2,2] # Position in the matrix [3,3]


    if ((axis=="ZXZ")or(axis=="zxz")): # Condition for the ZXZ axis
        if ((r33!=1)or(r33!=-1)):
            theta = arctan2((sqrt(1-(r33**2))), r33)
            phi = arctan2(r13, -r23)
            psi = arctan2(r31, r32)

        if sol: # Solution 2 in the case of "Theta"
            theta = arctan2(-sqrt(1-(r33**2)), r33)
            phi = arctan2(-r13, r23)
            psi = arctan2(-r31, -r32)

        if r33==1:
            theta = 0
            phi = 0
            psi = arctan2(r21, r11)

        if r33==-1:
            theta = pi
            phi = 0
            psi  = arctan2(-r21, -r11)

    if ((axis=="ZYZ")or(axis=="zyz")):  # Condition for the ZYZ axis
        if ((r33!=1)or(r33!=-1)):
            theta = arctan2((sqrt(1-(r33**2))), r33)
            phi = arctan2(r23, r13)
            psi = arctan2(r32, -r31)

        if sol: # Solution 2 in the case of "Theta"
            theta = arctan2(-sqrt(1-(r33**2)), r33)
            phi = arctan2(-r23, -r13)
            psi = arctan2(-r32, r31)

        if r33==1:
            theta = 0
            phi = 0
            psi = arctan2(r21, r11)

        if r33==-1:
            theta = pi
            phi = 0
            psi = arctan2(-r21, -r11)

    if ((axis=="XZX")or(axis=="xzx")): # Condition for the XZX axis
        if ((r11!=1)or(r11!=-1)):
            theta = arctan2((sqrt(1-(r11**2))), r11)
            phi = arctan2(r31, r21)
            psi = arctan2(r13, -r12)

        if sol: # Solution 2 in the case of "Theta"
            theta = arctan2(-sqrt(1-(r11**2)), r11)
            phi = arctan2(-r31, -r21)
            psi = arctan2(-r13, r12)

        if r11==1:
            theta = 0
            phi = 0
            psi = arctan2(r32, r22)

        if r11==-1:
            theta = pí
            phi = 0
            psi = arctan2(-r32, r22)

    if ((axis=="XYX")or(axis=="xyx")): # Condition for the XYX axis
        if ((r11!=1)or(r11!=-1)):
            theta = arctan2((sqrt(1-(r11**2))), r11)
            phi = arctan2(r21, -r31)
            psi = arctan2(r12, r13)

        if sol: # Solution 2 in the case of "Theta"
            theta = arctan2(-sqrt(1-(r11**2)), r11)
            phi = arctan2(-r21, r31)
            psi = arctan2(-r12, -r13)

        if r11==1:
            theta = 0
            phi = 0
            psi = arctan2(r32, r22)

        if r11==-1:
            theta = pi
            phi = 0
            psi = arctan2(-r32, -r22)

    if ((axis=="YZY")or(axis=="yzy")): # Condition for the YZY axis
        if ((r22!=1)or(r22!=-1)):
            theta = arctan2(sqrt(1-(r22**2)), r22)
            phi = arctan2(r32, -r12)
            psi = arctan2(r23, r21)

        if sol: # Solution 2 in the case of "Theta"
            theta = arctan2(-sqrt(1-(r22**2)), r22)
            phi = arctan2(-r32, r12)
            psi = arctan2(-r23, -r21)

        if r22==1:
            theta = 0
            phi = 0
            psi = arctan2(r13, r33)

        if r22==-1:
            theta = pi
            phi = 0
            psi = arctan2(-r13, -r33)

    if ((axis=="YXY")or(axis=="yxy")): # Condition for the YXY axis
        if ((r22!=1)or(r22!=-1)):
            theta = arctan2((sqrt(1-(r22**2))), r22)
            phi = arctan2(r12, r32)
            psi = arctan2(r21, -r23)

        if sol: # Solution 2 in the case of "Theta"
            theta = arctan2(-sqrt(1-(r22**2)), r22)
            phi = arctan2(-r12, -r32)
            psi = arctan2(-r21, r23)

        if r22==1:
            theta = 0
            phi = 0
            psi = arctan2(r13, r33)

        if r22==-1:
            theta = pi
            phi = 0
            psi = arctan2(-r13, -r33)

    if deg: # Convert the angles to degrees
        return rad2deg(phi), rad2deg(theta), rad2deg(psi)

    return phi,theta,psi

def rot2RPY(R, deg = False, sol = False):
    """
    Calculates the Roll, Pitch, Yaw angles from a rotation matrix on the XYZ axis

    ** The angles must be given in radians by default **

    Important: The rotation matrix must be 3x3
    """

    r11 = R[0,0] # Position in the matrix [1,1]
    r12 = R[0,1] # Position in the matrix [1,2]
    r13 = R[0,2] # Position in the matrix [1,3]
    r21 = R[1,0] # Position in the matrix [2,1]
    r22 = R[1,1] # Position in the matrix [2,2]
    r23 = R[1,2] # Position in the matrix [2,3]
    r31 = R[2,0] # Position in the matrix [3,1]
    r32 = R[2,1] # Position in the matrix [3,2]
    r33 = R[2,2] # Position in the matrix [3,3]

    if ((r31!=1)or(r31!=-1)): # Conditions that the matrix must have
        theta = arctan2(r31, (sqrt(1-(r31**2))))
        phi = arctan2(r21, r11)
        psi = arctan2(r32, r33)

    if sol: # Solution 2 in the case of "Theta"
        theta = arctan2(r31, -sqrt(1-(r31**2)))
        phi = arctan2(-r21, -r11)
        psi = arctan2(-r32, -r33)

    if r31==1:
        theta = pi/2
        phi = 0
        psi = arctan2(-r21, r22)

    if r31==-1:
        theta = (3*pi)/2
        phi = 0
        psi = arctan2(r21, -r22)

    if deg: # Convert the angles to degrees
        return rad2deg(phi), rad2deg(theta), rad2deg(psi)

    return phi, theta, psi

def rot2axa(R, deg = False):
    """
    Calculates the axis / angle ratio from a rotation matrix

    ** The angles must be given in radians by default **

    Important: The rotation matrix must be 3x3
    """
    r11 = R[0,0] # Position in the matrix [1,1]
    r12 = R[0,1] # Position in the matrix [1,2]
    r13 = R[0,2] # Position in the matrix [1,3]
    r21 = R[1,0] # Position in the matrix [2,1]
    r22 = R[1,1] # Position in the matrix [2,2]
    r23 = R[1,2] # Position in the matrix [2,3]
    r31 = R[2,0] # Position in the matrix [3,1]
    r32 = R[2,1] # Position in the matrix [3,2]
    r33 = R[2,2] # Position in the matrix [3,3]

    theta = arccos((r11+r22+r33-1)/2)

    k = ((1/(2*sin(theta)))*(np.array([[r32-r23], [r13-r31], [r21-r12]])))

    if deg: # Convert the angles to degrees
        return rad2deg(theta), k

    return theta, k

def htmDH(a,al,d,t, deg=False ):
    """
    Calculates the homogeneous matrix with the Denavir - Hartenberg (DH) parameters

    ** The angles must be given in radians by default **
    """

    if deg: # If theta is given in degrees -> convert to radians
        al = deg2rad(al)
        t = deg2rad(t)
    cal = cos(al)
    sal = sin(al)
    ct = cos(t)
    st = sin(t)
    H = np.array([[ct, -st*cal, st*sal, a*ct],
                [st, ct*cal, -ct*sal, a*st],
                [0, sal, cal, d],
                [0, 0, 0, 1]])

    return H

if __name__=="__main__":
    #matrix = [[0.0669,-0.933,0.3536],[0.933,-0.669,-0.3536],[0.3536,0.3536,0.866]]
    #H = np.array(matrix)
    #print("RESULTADO: ", rot2axa(H, True))

    #DH1 = htmDH(5,0,10,45)
    #DH2 = htmDH(10,20,30,90)
    #DH3 = htmDH(25,10,9,2,True)
    
    print("RESULTADO: \n", rot(30,45,60,'zxz',True))
