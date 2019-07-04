import numpy as np
import matplotlib.pyplot as plt
import transformations as tr



def htmrot(theta, axis="z", deg=False):
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
    H = np.array([[ct, -st, 0, 0],
                  [st, ct, 0, 0],
                  [0, 0, 1, 0], 
                  [0, 0, 0, 1]])
    return H


def htmtra(q):
    """
    Calculates the translation matrix
    
    *q* : list
		List -> [qx,qy,qz]
    """
    H = np.array([[1, 0, 0, q[0]],
                  [0, 1, 0, q[1]],
                  [0, 0, 1, q[2]], 
                  [0, 0, 0, 1]])
    return H



class RigidBody(object):
    """
    Define un cuerpo rígido en el plano mediante un sistema de partículas (puntos) 
    que lo conforman.
    """
    def __init__(self,points):
        self._points = points # Puntos que conforman el sólido rígido
        self.Hs = [np.eye(4),] # Matrices de transformación
    
    @property
    def points(self):
        _points = [] 
        H = self.H # Aplicando MTH
        for p in self._points:
            Q = np.array([p[0],p[1],0,1]) # Coords. Homog.
            _points.append(np.dot(H,Q))
        return _points
    
    @property
    def H(self):
        _h = np.eye(4)
        for _mth in self.Hs:
            _h = np.dot(_h,_mth)
        return _h
    
    def rotate(self,angle,axis="z"):
        """
        Rota el cuerpo rígido un ángulo determinado alrededor 
        de un eje coordenado.
        """
        if axis=="z":
            R = htmrot(angle, axis) # Aplicando rotación
        self.Hs.append(R)
    
    def move(self,q):
        """
        Traslada el cuerpo rígido un vector q
        """
        T = htmtra(q) # Aplicando traslación
        self.Hs.append(T)
        
    def scale(self,sf):
        """
        Escala el cuerpo rígido
        """
        #S = self.SM(sf) # Aplicando escalado
        #self.Hs.append(S)
        pass

    def SM(self,sf):
        M = np.array([[sf,0,0,0],
                      [0,sf,0,0],
                      [0,0,sf,0],
                      [0,0,0,sf]])
        return M
        
    def draw(self,color="r"):
        """
        Dibuja el cuerpo rígido en sus estatus actual
        """
        X,Y = [],[]
        for p in self.points:
            X.append(p[0])
            Y.append(p[1])
        plt.fill(X,Y,color)
        plt.axis('equal')
        plt.grid(ls="--")
        


if __name__=="__main__":
    from numpy import pi
    points = [(0,0),(0,1),(2,1),(2,0)]
    rx = RigidBody(points)
    rx.draw("y")
    rx.rotate(pi/3)
    rx.draw("r")
    rx.move([5,10,5])
    rx.draw("g")
    plt.show()
