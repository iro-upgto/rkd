#mathematical_algorithms
from numpy import *
import numpy as np

def newton_raphson(J, b, X0, eps=1e-6):
    """
    Calculates the numerical method Newton - Raphson
    """

    k=1
    b=b.evalf()
    while True:
        x=((J.X0).inv()*bX0).evalf()
        if x.norm()<eps: break
        for jj,ky  in enumerate(X0):
            X0[ky] += (x[jj]).evalf()
        k += 1
    return X0, x, k

def m_mult(*matrices):
    """
    Calculates the matrix resulting from a series of matrix multiplications
    """
    
    f,_ = ( matrices[0] ).shape
    A = np.eye(f)
    for matriz in matrices:
        A = np.dot(A, matriz)
    return A

if __name__=="__main__":

    A = np.array([[1, 2, 3],
                  [1, 2, 3], 
                  [1, 2, 3]])

    B = np.array([[1, 2, 3],
                  [1, 2, 3], 
                  [1, 2, 3]])

    C = np.array([[1, 2, 3],
                  [1, 2, 3], 
                  [1, 2, 3]])

    print("RESULTADO: ", m_mult(A, B, C, C, B, A))