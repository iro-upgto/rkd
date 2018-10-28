"""
This module has been designed for academic purposes, using SymPy as base library. 
It's easy to check that SymPy is slower than NumPy specially in matrix algebra, 
however SymPy is more convenient to use as didactic tool due to the given facilities 
as the symbolic manipulation, calculation of partial and ordinary derivatives, 
matricial multiplication using asterisk symbol, "init_printing" function and so on.
"""
from sympy import *
from sympy.matrices import Matrix,eye


def deg2rad(theta):
    """ Convert degrees to radians """
    return ( theta*(pi/180) ).evalf()


def rad2deg(theta):
    """ Convert radians to degrees """
    return ( theta*(180/pi) ).evalf()
    
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
    """
    nrow,ncol = R.shape
    if (nrow == ncol == 3) and isorthonormal(R):
        return True
    return False
    
def isorthonormal(R):
    """
    Check if R is orthonormal
    """
    nrow,ncol = R.shape
    for i,j in combinations(range(ncol), 2):
        if ( R[:,i].dot(R[:,j]) ).simplify() != 0:
            return False
    for i in range(ncol):
        if R[:,i].norm().simplify() != 1:
            return False
    return True



if __name__=="__main__":
    pass
