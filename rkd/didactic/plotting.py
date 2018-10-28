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
from rkd.didactic.transformations import *


def plot_euler(phi,theta,psi,seq="zxz"):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    draw_uvw(eye(4), ax)
    R1 = 
    draw_uvw(H, ax)
    plt.show()
    
    
def draw_uvw(H,ax,sz=1):
    u = H[:3,0]
    v = H[:3,1]
    w = H[:3,2]
    if ishtm(H):
    o = H[:3,3]
    L = sz/5
    ax.quiver(o[0],o[1],o[2],u[0],u[1],u[2], color="r", length=L)
    ax.quiver(o[0],o[1],o[2],v[0],v[1],v[2], color="g", length=L)
    ax.quiver(o[0],o[1],o[2],w[0],w[1],w[2], color="b", length=L)


if __name__=="__main__":
    plot_euler(pi/4,pi/4,pi/4)
