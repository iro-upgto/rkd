"""

"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import operator, functools
from rkd.didactic.transformations import *
from sympy import *
from sympy.matrices import Matrix,eye
from rkd.abc import *

class Robot(object):
    """
    Define a robot-serial-arm given the Denavit-Hartenberg parameters
    """
    def __init__(self,*args):
        self.Ts = [] # Transformation matrices i to i-1
        self.type = [] # Joint type -> "r" revolute, "p" prismatic
        for k in args:
            self.Ts.append(dh(k[0],k[1],k[2],k[3])) # Compute Ti->i-1
            if len(k)>4:
                self.type.append(k[4])
            else:
                self.type.append('r')
        self.dof = len(args) # Degree of freedom
    
    def z(self,i):
        """
        z-dir of every i-Frame wrt 0-Frame
        """
        if i == 0: return Matrix([[0],[0],[1]])
        MTH = eye(4)
        for k in range(i):
            MTH = MTH*self.Ts[k]
        return MTH[:3,2]
    
    def p(self,i):
        """
        Position for every i-Frame wrt 0-Frame
        """
        if i == 0: return Matrix([[0],[0],[0]])
        MTH = eye(4)
        for k in range(i):
            MTH = MTH*self.Ts[k]
        return MTH[:3,3]
    
    @property
    def J(self):
        """
        Geometric Jacobian matrix
        """
        n = self.dof
        M_ = zeros(6,n)
        for i in range(self.dof):
            if self.type[i]=='r':
                jp = self.z(i).cross(self.p(n) - self.p(i))
                jo = self.z(i)
            else:
                jp = self.z(i)
                jo = zeros(3,1)
            jp = jp.col_join(jo)
            M_[:,i] = jp
        return simplify(M_)

    def J_i(self,i):
        """
        Geometric Jacobian matrix
        """
        n = i
        M_ = zeros(6,n)
        for i in range(n):
            if self.type[i]=='r':
                jp = self.z(i).cross(self.p(n) - self.p(i))
                jo = self.z(i)
            else:
                jp = self.z(i)
                jo = zeros(3,1)
            jp = jp.col_join(jo)
            M_[:,i] = jp
        return simplify(M_).evalf(6)

    def dof(self):
        return self.dof

    @property
    def T(self):
        """ 
        T_n^0 
        Homogeneous transformation matrix of N-Frame respect to Base-Frame
        """
        return simplify(functools.reduce(operator.mul, self.Ts))
        
    def Ti_0(self,i):
        return simplify(functools.reduce(operator.mul, self.Ts[:i+1]))
        
    def plot_diagram(self,vals):
        #return None
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        
        Ts = self.Ts
        points = []
        Ti_0 = []
        points.append(zeros(1,3))
        for i in range(self.dof):
            Ti_0.append(self.Ti_0(i).subs(vals))
            points.append((self.Ti_0(i)[:3,3]).subs(vals))
            
        X = [float(k[0]) for k in points]
        Y = [float(k[1]) for k in points]
        Z = [float(k[2]) for k in points]
        ax.plot(X,Y,Z, "o-", color="#778877", lw=3)
        ax.plot([0],[0],[0], "mo", markersize=6)
        ax.set_axis_off()
        ax.view_init(90,0)
        
        px,py,pz = float(X[-1]),float(Y[-1]),float(Z[-1])
        dim = max([px,py,pz])
        
        self.draw_uvw(eye(4),ax, dim)
        for T in Ti_0:
            self.draw_uvw(T, ax, dim)
            
        ax.set_xlim(-dim, dim)
        ax.set_ylim(-dim, dim)
        ax.set_zlim(-dim, dim)
        plt.show()
    
    def draw_uvw(self,H,ax,sz=1):
        u = H[:3,0]
        v = H[:3,1]
        w = H[:3,2]
        o = H[:3,3]
        L = sz/5
        ax.quiver(o[0],o[1],o[2],u[0],u[1],u[2],color="r", length=L)
        ax.quiver(o[0],o[1],o[2],v[0],v[1],v[2],color="g", length=L)
        ax.quiver(o[0],o[1],o[2],w[0],w[1],w[2],color="b", length=L)




class RigidBody2D(object):
    """
    Defines a rigid body through a series of points that 
    make it up.
    """
    def __init__(self,points):
        self._points = points # Points
        self.Hs = [eye(4),] # Transformation matrices
    
    @property
    def points(self):
        _points = []
        H = self.H #
        for p in self._points:
            Q = Matrix([p[0],p[1],0,1]) # Homogeneous coordinates
            _points.append(H*Q)
        return _points
    
    @property
    def H(self):
        _h = eye(4)
        for _mth in self.Hs:
            _h = _h*_mth
        return _h

    def rotate(self,angle):
        """
        Rota el cuerpo rígido un ángulo determinado alrededor 
        del eje coordenado z.
        """
        R = htmrot(angle, axis="z") # Aplicando rotación
        self.Hs.append(R)
    
    def move(self,q):
        """
        Traslada el cuerpo rígido un vector q
        """
        D = htmtra(q) # Aplicando traslación
        self.Hs.append(D)
        
    def scale(self,sf):
        """
        Escala el cuerpo rígido
        """
        # ~ S = self.scale_matrix(sf) # Aplicando escalado
        # ~ self.Hs.append(S)
        pass # nothing to do here

    def scale_matrix(self,sf):
        M = Matrix([[sf,0,0,0],
                      [0,sf,0,0],
                      [0,0,sf,0],
                      [0,0,0,sf]])
        return M
        
    def draw(self,color="r",kaxis=None):
        """
        Dibuja el cuerpo rígido en sus estatus actual
        """
        X,Y = [],[]
        cx,cy = self.get_centroid()
        for p in self.points:
            X.append(p[0])
            Y.append(p[1])
        plt.fill(X,Y,color,alpha=0.8)
        plt.plot(cx,cy,"r.")
        plt.axis('equal')
        plt.grid(ls="--")
        
        O = self.H[:3,3]
        U = self.H[:3,0]
        V = self.H[:3,1]
        plt.quiver(float(O[0]), float(O[1]), float(U[0]), float(U[1]), color="r", zorder=1000, scale=kaxis)
        plt.quiver(float(O[0]), float(O[1]), float(V[0]), float(V[1]), color="g", zorder=1001, scale=kaxis)

        
    def get_centroid(self):
        n = len(self.points)
        sx,sy = 0,0
        for point in self.points:
            sx += point[0]
            sy += point[1]
        cx = sx/n
        cy = sy/n
        return cx,cy



def test_robot():
    r = Robot((l1,0,0,t1), (l2,0,0,t2))
    r.plot_diagram({t1:pi/2, t2:pi/2, l1:100, l2:100})
    
    
def test_rb2():
    points = [(0,0),(3,0),(0,1)]
    rb = RigidBody2D(points)
    rb.draw("r")
    rb.move([10,0,0])
    rb.draw("g")
    rb.rotate(pi/2)
    rb.move([5,0,0])
    rb.draw("b")
    plt.show()
    print(rb.Hs)


if __name__=="__main__":
    H = Matrix([[0,0,1,0], [0,-1,0,0], [1,0,0,0], [0,0,0,1]])
    test_rb2()
