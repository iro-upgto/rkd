# Transformations
import numpy as np
from numpy import arctan2,sqrt


def rotz(theta, deg=False):
    """
    Calculates the rotation matrix about the z-axis

    *theta* : float or int
        Rotation angle (given in radians by default)

    *deg* : bool
        ¿Is theta given in degrees?
    """
    if deg: # If theta is given in degrees -> convert to radians
        theta = theta*np.pi/180
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
        theta = theta*np.pi/180
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
        theta = theta*np.pi/180
    ct = np.cos(theta)
    st = np.sin(theta)
    R = np.array([[ct,0,st],
                  [0,1,0],
                  [-st,0,ct]])
    return R

def rot2eul(R):
    """
    Calculate the Euler angles from a rotation matrix

    Angles given in radians by default

    Important: The rotation matrix must be 3x3
    """
    
    r11 = R[0,0]
    r12 = R[0,1]
    r13 = R[0,2]
    r21 = R[1,0]
    r22 = R[1,1]
    r23 = R[1,2]
    r31 = R[2,0]
    r32 = R[2,1]
    r33 = R[2,2]

    if ((r33!=1)or(r33!=-1)):
        theta = arctan2((sqrt(1-(r33**2))),r33)
        phi = arctan2(r13,-r23)
        psi = arctan2(r31,r32)
    if ((r33==1)or(r33==1)):
        theta = 0
        phi = 0
        psi = arctan2(r21,r11)
    return theta,phi,psi

if __name__=="__main__":

    R = np.array([[0.8666,-0.5,0],
                  [0.25,0.433,-0.8666],
                  [0.433,0.75,0.5]])

    print(rot2eul(R))