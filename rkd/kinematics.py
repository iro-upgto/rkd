#Kinematics
import numpy as np
from numpy import *
from math import *
from rkd.util import *
from rkd.transformations import *
from rkd.mathematical_algorithms import *
from scipy.optimize import *

def jacobian(*args):
    """
    Calculates the Jacobian matrix to determine the differential kinematics of manipulators / robots
    """
    Ts = []
    type = []
    for k in args:
        Ts.append(htmDH(k[0],k[1],k[2],k[3]))
        if len(k)>4:
            type.append(k[4])
        else:
            type.append('r')
    dof = len(args)

    n = dof
    M_ = np.zeros((6,n))
    for i in range(dof):
        if type[i]=='r':
            jp = np.cross(z(i,Ts), (p(n,Ts) - p(i,Ts)))
            jo = z(i,Ts)
        else:
            jp = z(i,Ts)
            jo = np.zeros((3,1))
        jp = np.row_stack((jp.reshape(3,-1),jo.reshape(3,-1)))
        M_[:,i] = jp.flatten()
    return M_

def z(i,Ts):
    """
    z-dir of every i-Frame wrt 0-Frame
    """
    if i == 0: return np.array([0,0,1])
    MTH = np.eye(4)
    for k in range(i):
        MTH = m_mult(MTH, Ts[k])
    return MTH[:3,2]

def p(i,Ts):
    """
    Position for every i-Frame wrt 0-Frame
    """
    if i == 0: return np.array([0,0,0])
    MTH = np.eye(4)
    for k in range(i):
        MTH = m_mult(MTH,Ts[k])
    return MTH[:3,3]

#Kinematics inverse

def inverse_kinematics(J, b, x0, number_of_variables, matrixJ, matrixF, deg = False,eps = 1e-6):
    """
    Calculates the inverse kinematics from a numerical method

    ** Solved from the numerical method 'Newton - Raphson' **

    *deg* : bool
        ¿Is theta given in degrees?

    IMPORT: You must create your two functions of J (Jacobian), b (initial values ​​evaluated)
    """
    k = 1 #iterations
    num_var = number_of_variables
    while True:
        x = np.linalg.solve(J(matrixJ, x0, num_var), -b(matrixF, x0, num_var))
        if norm(x) < eps: break
        x0 += x
        k += 1 #Increase of iterations counter
    if deg:
        return rad2deg(x0), x, k
    return x0, x, k

def j1(J, x0, number_of_variables):
    """
    Calculates the Jacobian matrix evaluated with the initial values
    """
    var = ['var1', 'var2', 'var3', 'var3', 'var4', 'var5', 'var6', 'var7', 'var8', 'var9']
    num_var = number_of_variables

    if num_var == '1':
        var1 = var[0]
        if var1 == 'var1':
            var1 = x0            
    if num_var == '2':
        var1 = var[0]
        var2 = var[1]
        if ((var1 == 'var1') and (var2 == 'var2')):
            var1, var2 = x0
    if num_var == '3':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3')):
            var1, var2, var3 = x0
    if num_var == '4':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        var4 = var[3]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4')):
            var1, var2, var3, var4 = x0
    if num_var == '5':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        var4 = var[3]
        var5 = var[4]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5')):
            var1, var2, var3, var4, var5 = x0
    if num_var == '6':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        var4 = var[3]
        var5 = var[4]
        var6 = var[5]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5') and (var6 == 'var6')):
            var1, var2, var3, var4, var5, var6 = x0
    if num_var == '7':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        var4 = var[3]
        var5 = var[4]
        var6 = var[5]
        var7 = var[6]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5') and (var6 == 'var6') and (var7 == 'var7')):
            var1, var2, var3, var4, var5, var6, var7 = x0
    if num_var == '8':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        var4 = var[3]
        var5 = var[4]
        var6 = var[5]
        var7 = var[6]
        var8 = var[7]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5') and (var6 == 'var6') and (var7 == 'var7')  and (var8 == 'var8')):
            var1, var2, var3, var4, var5, var6, var7, var8 = x0
    if num_var == '9':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        var4 = var[3]
        var5 = var[4]
        var6 = var[5]
        var7 = var[6]
        var8 = var[7]
        var9 = var[8]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5') and (var6 == 'var6') and (var7 == 'var7')  and (var8 == 'var8') and (var9 == 'var9')):
            var1, var2, var3, var4, var5, var6, var7, var8, var9 = x0
    J = '['+J+']'
    J = eval(J)
    J = np.array(J)    
    F = J
    return J

def b1(F, x0, number_of_variables):
    """
    calculates the approximation of vector 'bi' on the method 'Newton - Rapshon'
    """
    var = ['var1', 'var2', 'var3', 'var3', 'var4', 'var5', 'var6', 'var7', 'var8', 'var9']
    num_var = number_of_variables

    if num_var == '1':
        var1 = var[0]
        if var1 == 'var1':
            var1 = x0            
    if num_var == '2':
        var1 = var[0]
        var2 = var[1]
        if ((var1 == 'var1') and (var2 == 'var2')):
            var1, var2 = x0
    if num_var == '3':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3')):
            var1, var2, var3 = x0
    if num_var == '4':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        var4 = var[3]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4')):
            var1, var2, var3, var4 = x0
    if num_var == '5':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        var4 = var[3]
        var5 = var[4]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5')):
            var1, var2, var3, var4, var5 = x0
    if num_var == '6':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        var4 = var[3]
        var5 = var[4]
        var6 = var[5]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5') and (var6 == 'var6')):
            var1, var2, var3, var4, var5, var6 = x0
    if num_var == '7':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        var4 = var[3]
        var5 = var[4]
        var6 = var[5]
        var7 = var[6]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5') and (var6 == 'var6') and (var7 == 'var7')):
            var1, var2, var3, var4, var5, var6, var7 = x0
    if num_var == '8':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        var4 = var[3]
        var5 = var[4]
        var6 = var[5]
        var7 = var[6]
        var8 = var[7]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5') and (var6 == 'var6') and (var7 == 'var7')  and (var8 == 'var8')):
            var1, var2, var3, var4, var5, var6, var7, var8 = x0
    if num_var == '9':
        var1 = var[0]
        var2 = var[1]
        var3 = var[2]
        var4 = var[3]
        var5 = var[4]
        var6 = var[5]
        var7 = var[6]
        var8 = var[7]
        var9 = var[8]
        if ((var1 == 'var1') and (var2 == 'var2') and (var3 == 'var3') and (var4 == 'var4') and (var5 == 'var5') and (var6 == 'var6') and (var7 == 'var7')  and (var8 == 'var8') and (var9 == 'var9')):
            var1, var2, var3, var4, var5, var6, var7, var8, var9 = x0
    F = eval(F)
    F = np.array(F)    
    F = F
    return F

def ec(x):
    return np.array([(x[0]*cos(x[1])*sin(x[2])), (x[0]*sin(x[1])*sin(x[2])), (10-(x[0]*cos(x[2])))])
if __name__=="__main__":

    #x0 = np.array([[80, pi/6, pi/4]])
    #J = '[-100*sin(var1) - 100*sin(var1 + var2), -100*sin(var1 + var2)], [100*cos(var1) + 100*cos(var1 + var2), 100*cos(var1 + var2)]'
    #F = '[100*cos(var1) + 100*cos(var1 + var2) - 100, 100*sin(var1) + 100*sin(var1 + var2) - 100]'
    
    #print(inverse_kinematics(j1, b1, x0, '2', J, F, True))

    #print(root(ec, x0))
    DH1 = "0,1.570796327, 100, 0.785398163, 'r'"
    DH2 = "0, 1.570796327, 0, 1.047197551, 'r'"
    DH3 = "0, 0, 150, 0, 'p'"
    DH1 = eval(DH1)
    DH2 = eval(DH2)
    DH3 = eval(DH3)
    print(jacobian(DH1, DH2, DH3))