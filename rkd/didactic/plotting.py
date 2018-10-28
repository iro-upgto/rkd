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
from rkd.didactic.util import *


def plot_euler(phi,theta,psi,seq="zxz"):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    if seq in ("zxz","ZXZ"):
        R1 = rotz(phi)
        R2 = R1*rotx(theta)
        R3 = R2*rotz(psi)
    draw_uvw(eye(4), ax, "k", 8)
    draw_uvw(R1, ax, "r", 6)
    draw_uvw(R2, ax, "g", 4)
    draw_uvw(R3, ax, "b", 3)
    ax.set_xlim([-1,1])
    ax.set_ylim([-1,1])
    ax.set_zlim([-1,1])
    ax.set_aspect("equal")
    ax.axis('off')
    plt.show()
    
    
    
    
    
def draw_uvw(H,ax,color=("r","g","b"),sz=1):
    u = H[:3,0]
    v = H[:3,1]
    w = H[:3,2]
    if ishtm(H):
        o = H[:3,3]
    else:
        o = Matrix([0,0,0])
    L = sz/5
    if isinstance(color,str):
        colorl = (color,color,color)
    else:
        colorl = color
    ax.quiver(o[0],o[1],o[2],u[0],u[1],u[2], color=colorl[0], 
              length=L, arrow_length_ratio=0.2)
    ax.quiver(o[0],o[1],o[2],v[0],v[1],v[2], color=colorl[1], 
              length=L, arrow_length_ratio=0.2)
    ax.quiver(o[0],o[1],o[2],w[0],w[1],w[2], color=colorl[2], 
              length=L, arrow_length_ratio=0.2)


if __name__=="__main__":
    plot_euler(pi/3,pi/3,pi/3)
