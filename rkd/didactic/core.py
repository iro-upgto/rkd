"""
This module has been designed for academic purposes, using SymPy as base library. 
It's easy to check that SymPy is slower than NumPy specially in matrix algebra, 
however SymPy is more convenient to use as didactic tool due to the given facilities 
as the symbolic manipulation, calculation of partial and ordinary derivatives, 
matricial multiplication using asterisk symbol, "init_printing" function and so on.
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import operator, functools
from sympy import *
from sympy.matrices import Matrix,eye
from rkd.abc import *
init_printing(use_latex=True)

# ~ ==========================================
# ~ Conversions
# ~ ==========================================

def deg2rad(theta):
    """ Convert degrees to radians """
    return theta*(pi/180)
    
def rad2deg(theta):
    """ Convert radians to degrees """
    return ( theta*(180/pi) ).evalf()
    
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
    test_robot()
    # ~ test_rb2()
