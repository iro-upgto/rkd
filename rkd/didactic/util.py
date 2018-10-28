"""
This module has been designed for academic purposes, using SymPy as base library. 
It's easy to check that SymPy is slower than NumPy specially in matrix algebra, 
however SymPy is more convenient to use as didactic tool due to the given facilities 
as the symbolic manipulation, calculation of partial and ordinary derivatives, 
matricial multiplication using asterisk symbol, "init_printing" function and so on.
"""
from sympy import *
from sympy.matrices import Matrix,eye

# ~ ==========================================
# ~ Conversions
# ~ ==========================================

def deg2rad(theta):
    """ Convert degrees to radians """
    return ( theta*(pi/180) ).evalf()


def rad2deg(theta):
    """ Convert radians to degrees """
    return ( theta*(180/pi) ).evalf()
    
def ishtm(H):
    nrow,ncol = H.shape
    if nrow == ncol == 4:
        return True
    return False

def isrot(R):
    nrow,ncol = R.shape
    if nrow == ncol == 3:
        return True
    return False



if __name__=="__main__":
    pass
