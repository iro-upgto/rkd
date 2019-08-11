"""

"""
from sympy import pi,sin,cos,tan
from sympy.matrices import Matrix,eye
from itertools import combinations
# ~ from scipy.spatial import Delaunay, ConvexHull
import numpy as np


def deg2rad(theta, evalf=True):
    """
    Convert degrees to radians 
    
    Parameters
    ----------
    
    theta : float, int, symbolic
    
    Returns
    -------
    
    theta_rad : symbolic
    """
    if evalf:
        theta_rad = ( theta*(pi/180) ).evalf()
    else:
        theta_rad = theta*(pi/180)
    return theta_rad


def rad2deg(theta, evalf=True):
    """
    Convert radians to degrees 
    
    Parameters
    ----------
    
    theta : float, int, symbolic
    
    Returns
    -------
    
    theta_deg : symbolic
    """
    if evalf:
        theta_deg = ( theta*(180/pi) ).evalf()
    else:
        theta_deg = theta*(180/pi)
    return theta_deg
    
def ishtm(H):
    """
    Is H a homogeneous transformation matrix ?
    """
    nrow,ncol = H.shape
    if nrow == ncol == 4:
        return True
    return False

def isrot(R):
    """
    Is R a rotation matrix ?
    
    Parameters
    ----------
    
    R : `sympy.matrices.dense.MutableDenseMatrix`
    
    Returns
    -------
    
    False or True
    
    """
    nrow,ncol = R.shape
    if (nrow == ncol == 3) and isorthonormal(R):
        return True
    return False
    
def isorthonormal(R):
    """
    Check if R is orthonormal
    
    Parameters
    ----------
    
    R : `sympy.matrices.dense.MutableDenseMatrix`
    
    Returns
    -------
    
    False or True
    
    """
    _,ncol = R.shape
    for i,j in combinations(range(ncol), 2):
        if ( R[:,i].dot(R[:,j]) ).simplify() != 0:
            return False
    for i in range(ncol):
        if R[:,i].norm().simplify() != 1:
            return False
    return True


if __name__=="__main__":
    pass
