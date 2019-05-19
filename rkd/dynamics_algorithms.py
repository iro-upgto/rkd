import sympy
import numpy as np
from sympy import *
from sympy.matrices import Matrix, eye
from rkd.abc import *

def Dh(a,alpha,d,theta):
    """
    Calculates Denavit-Hartenberg matrix given the four parameters.

    Parameters
    ----------
    a : int, float or symbol
        DH parameter
    alpha : int, float or symbol
        DH parrameter
    d : int, float or symbol
        DH parameter
    theta : int, float or symbol
        DH parameter

    Returns
    -------
    H : :class:`sympy.matrices.dense.MutableDenseMatrix`
        Denavit-Hartenberg matrix (4x4)

    Examples
    --------
    >>> dh(100,pi/2,50,pi/2)
    ⎡0  0  1   0 ⎤
    ⎢            ⎥
    ⎢1  0  0  100⎥
    ⎢            ⎥
    ⎢0  1  0  50 ⎥
    ⎢            ⎥
    ⎣0  0  0   1 ⎦
    """
    H = Matrix([[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta)],
                  [sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta)],
                  [0,sin(alpha),cos(alpha),d],
                  [0,0,0,1]])
    return H

def z(i,Ts):
    """
    z-dir of every i-Frame wrt 0-Frame
    """
    if i == 0: return Matrix([[0],[0],[1]])
    MTH = eye(4)
    for k in range(i):
        MTH = MTH*Ts[k]
    return MTH[:3,2]

def p(i,Ts):
    """
    Position for every i-Frame wrt 0-Frame
    """
    if i == 0: return Matrix([[0],[0],[0]])
    MTH = eye(4)
    for k in range(i):
        MTH = MTH*Ts[k]
    return MTH[:3,3]

def jac(*args):
    """
    Calculates the Jacobian matrix to determine the differential kinematics of manipulators / robots
    """
    Ts = []
    type = []
    for k in args:
        Ts.append(Dh(k[0],k[1],k[2],k[3]))
        if len(k)>4:
            type.append(k[4])
        else:
            type.append('r')
    dof = len(args)

    n = dof
    M_ = zeros(6,n)
    for i in range(dof):
        if type[i]=='r':
            jp = z(i,Ts).cross(p(n,Ts) - p(i,Ts))
            jo = z(i,Ts)
        else:
            jp = z(i,Ts)
            jo = zeros(3,1)
        jp = jp.col_join(jo)
        M_[:,i] = jp
    return M_

def DH(*args):
    Ts = []
    H = eye(4)
    for k in args:
        Ts.append(Dh(k[0], k[1], k[2], k[3]))
    for i in range(len(args)):
        H = H*Ts[i]

    return H

def potential_energy(RT, size_linear_velocity, size_table2, direction_of_gravity = 'y'):
    m, j = [], []
    for i in range(size_table2):
        m.append(eval('m'+str(int("".join(str(i+1))))))
        j.append(eval('J'+str(int("".join(str(i+1))))))
    
    u, U, H = [], [], []
    for i in range(len(RT)):
            H.append(DH(*RT[i]))
    for i in range(len(RT)):
        if direction_of_gravity == 'x' or direction_of_gravity == 'X':
            u.append(H[i][:1,3])
        elif direction_of_gravity == 'y' or direction_of_gravity == 'Y':
            u.append(H[i][1:2,3])
        elif direction_of_gravity == 'z' or direction_of_gravity == 'Z':
            u.append(H[i][2:3,3])
    for i in range(size_linear_velocity):
        U.append((m[i]*g)*u[i])
    return simplify(np.sum(U[:], axis = 0))

def check_params(r1,r2,r3,r4,r5,r6):
    R = []
    for r in [r1,r2,r3,r4,r5,r6]:
        if r != '':
            R.append(eval(r))
    return R

def Lagrangian(params_SN_SM = ('r1','r2','r3','r4','r5','r6'), params_CN_SM = ('r1','r2','r3','r4','r5','r6'), mass = ('M1', 'M2', 'M3', 'M4', 'M5', 'M6'), j = ('j1', 'j2', 'j3', 'j4', 'j5', 'j6'), direction_of_gravity = 'y'):
    #*********************************************************************************************************************************************
    #          TABLE 1 (Parámetros DH sin considerar los sistemas de masas)
    #*********************************************************************************************************************************************
    t1r1, t1r2, t1r3, t1r4, t1r5, t1r6 = params_SN_SM[0], params_SN_SM[1], params_SN_SM[2], params_SN_SM[3], params_SN_SM[4], params_SN_SM[5]
    #*********************************************************************************************************************************************
    #       TABLE 2 (Parámetros DH considerando los sistemas de masas)
    #*********************************************************************************************************************************************
    t2r1, t2r2, t2r3, t2r4, t2r5, t2r6 = params_CN_SM[0], params_CN_SM[1], params_CN_SM[2], params_CN_SM[3], params_CN_SM[4], params_CN_SM[5]
    #*********************************************************************************************************************************************
    #       TABLE OF MASS
    #*********************************************************************************************************************************************
    M1, M2, M3, M4, M5, M6 = mass[0], mass[1], mass[2], mass[3], mass[4], mass[5]
    #*********************************************************************************************************************************************
    #       TABLES OF J
    #*********************************************************************************************************************************************
    j1, j2, j3, j4, j5, j6 = j[0], j[1], j[2], j[3], j[4], j[5]
    Table1 = check_params(t1r1, t1r2, t1r3, t1r4, t1r5, t1r6)
    Table2 = check_params(t2r1, t2r2, t2r3, t2r4, t2r5, t2r6)
    M = check_params(M1, M2, M3, M4, M5, M6)
    J = check_params(j1, j2, j3, j4, j5, j6)
    R1 = len(Table1)
    R2 = len(Table2)
    Rm = len(M)
    RJ = len(J)
    if R2 == Rm == RJ:
        #*********************************************************************************************************************************************
        #       JACOBIANO APPLIED IN A GENERAL MANNER
        #*********************************************************************************************************************************************
        JT = []
        if R1 >= R2:
            Table1 = Table1[0:R2-1]
            R1 = R2-1
        if  R2 == 1:
            R, RT = [], []
            R.append(Table2[0])
            jaco = jac(*R)
            JT.append(jaco)
            RT.append([Table2[0]])
        elif R2 > 1:
            RT = []
            for j in range(R2):
                R, k, i = [], j, R1
                k -= R2
                i += k+1
                if i <= R1:
                    for t in range(i):
                        R.append(Table1[t])
                R.append(Table2[j])
                RT.append(R)
            for i in range(len(RT)):
                JT.append(jac(*RT[i]))
        
        # print('**********************************************')
        # print(RT)
        # print('**********************************************')

        # print((Dh(*Table1[0]))*(Dh(*Table2[1]))) #OBTENER DE AQUÍ LA ENERGÍA POTENCIAL
        # print(JT) #OBTENER DE AQUÍ LA ENERGÍA POTENCIAL (PRIMER FILA COINCIDE)
        qp = []
        for i in range(R2):
            qp.append(eval('q'+str(int("".join(str(i+1))))+'p'))

        Jp = []
        for i in range(len(JT)):
            for j in range(i+1):
                Jp.append(JT[i][:,j]*qp[j])
        
        # print(jt)

        JP = []
        JP.append(Jp[0])
        i = 1
        l, k, z, a = 0, 2, 2, 1
        for j in range(1,len(JT)):
            JP.append(Matrix(np.sum(Jp[i:j+k], axis = 0)))
            i += j+a
            k += z
            z += 1

        # print('**********************************************')
        # print(JP)
        # print('**********************************************')

        m, j = [], []
        for i in range(R2):
            m.append(eval('m'+str(int("".join(str(i+1))))))
            j.append(eval('J'+str(int("".join(str(i+1))))))

        #*********************************************************************************************************************************************
        #       OBTAINING LINEAR AND ANGULAR SPEEDS
        #*********************************************************************************************************************************************
        v, w = [], []
        for i in range(len(JP)):
            v.append(simplify(JP[i][:3,0:i+1]))
            w.append(simplify(JP[i][3:,0:i+1]))

        # print('=========================')
        # print(v)
        # print('=========================')

        #*********************************************************************************************************************************************
        #           KINEMATIC ENERGY
        #*********************************************************************************************************************************************
        k = []
        for i in range(len(v)):
            k.append(simplify((0.5*m[i]*v[i].T*v[i])+(0.5*j[i]*w[i].T*w[i])))
        K = simplify(np.sum(k[:], axis = 0))

        # print('**********************************************')
        # print('KINEMATIC ENERGY')
        # print('**********************************************')
        # print(K)

        #*********************************************************************************************************************************************
        #          POTENTIAL ENERGY
        #*********************************************************************************************************************************************
        U = potential_energy(RT, len(v), R2)

        # print('**********************************************')
        # print(RT)
        # print('**********************************************')
        # print(H)
        # print('**********************************************')
        # print('POTENCIAL AQUI')
        # print(U)
        # print('**********************************************')
        # print('POTENCIAL FUNCION')
        # print(potential_energy(RT, len(v), R2))
        # print('**********************************************')
        # print('Lagrangian:')
        # # print(K - U)
        # print('**********************************************')

    return simplify(K-U), len(JT)

def check_lists(p1, p2, p3, p4, p5, p6):
    R = []
    for r in [p1, p2, p3, p4, p5, p6]:
        if r != []:
            R.append(r)
    return R

def sorter_of_the_dynamic_modeling(tau_n, dof):
    q, qp, qpp = [], [], []
    for i in range(dof):
        q.append(eval('q'+str(int("".join(str(i+1))))))
        qp.append(eval('q'+str(int("".join(str(i+1))))+'p'))
        qpp.append(eval('q'+str(int("".join(str(i+1))))+'p'+'p'))

    #*******************************************************
    #              CHECK TAU VECTOR
    #*******************************************************
    q_pp, Gravity = [], []
    for i in range(dof):
        qpp1, qpp2, qpp3, qpp4, qpp5, qpp6 = [], [], [], [], [], []
        for sumando in tau_n[i][0].args:
            for fc in sumando.args:
                if g == fc:
                    Gravity.append(sumando)
                if q1pp == fc:
                    qpp1.append(sumando)
                elif q2pp == fc:
                    qpp2.append(sumando)
                elif q3pp == fc:
                    qpp3.append(sumando)
                elif q4pp == fc:
                    qpp4.append(sumando)
                elif q5pp == fc:
                    qpp5.append(sumando)
                elif q6pp == fc:
                    qpp6.append(sumando)
        for r in [qpp1, qpp2, qpp3, qpp4, qpp5, qpp6]:
            if r != []:
                q_pp.append(r)
    
    r = []
    for i in range(len(q_pp)):
        r.append([np.sum(q_pp[i][:], axis = 0)])
    
    i, res = 0, []
    while i < len(r):
        for j in range(dof):
            res.append(Matrix([simplify(r[i][0]/qpp[j])]))
            i += 1
    #*******************************************************
    #           OBTAINING THE "INERTIA MATRIX"
    #*******************************************************
    B, k = ones(dof, dof), 0
    while k < len(r):
        for i in range(dof):
            for j in range(dof):
                B[i,j] = simplify(B[i,j]*res[k])
                k += 1

    #*******************************************************
    #           OBTAINING THE "CORIOLIS MATRIX"
    #*******************************************************
    C, p = ones(dof**3, 1), 0
    sizeC,_ = C.shape
    while p < sizeC:
        for k in range(dof):
            for i in range(dof):
                for j in range(dof):
                    C[p,0] = C[p,0]*0.5*(B[i,j].diff(q[k]) + B[i,k].diff(q[j]) - B[j,k].diff(q[i]))*qp[k]
                    p += 1
    
    Cor, i = ones(dof**2,1), 0
    sizeCor,_ = Cor.shape
    if dof == 1: Cor[0,0] = simplify(C[0,0])
    if dof > 1:
        while i < sizeCor:
            Cor[i,0] = simplify(Cor[i,0]*C[i,0])
            for k in range(1,dof):
                Cor[i,0] = simplify(Cor[i,0]+C[i+(k*dof**2),0])
            i += 1

    C, k = ones(dof, dof), 0
    while k < len(Cor):
        for i in range(dof):
            for j in range(dof):
                C[i,j] = simplify(C[i,j]*Cor[k])
                k += 1

    G, k = ones(dof, 1), 0
    while k < len(Gravity):
        for i in range(dof):
            G[i,0] = simplify(G[i,0]*Gravity[k])
            k += 1

    return B, C, G

def derivates_and_sorter(lagrangian, dof):
    diffq, diffqp, difft, q, qp, qpp, taus = [], [], [], [], [], [], []
    for i in range(dof):
        q.append(eval('q'+str(int("".join(str(i+1))))))
        qp.append(eval('q'+str(int("".join(str(i+1))))+'p'))
        qpp.append(eval('q'+str(int("".join(str(i+1))))+'p'+'p'))
        #      Necessary derivates for dynamic modeling
        diffq.append(simplify(lagrangian.diff(q[i])))
        diffqp.append(simplify(lagrangian.diff(qp[i])))
        difft.append(simplify(diffqp[i].diff(t)))
        taus.append(simplify(difft[i]-diffq[i]))

    B, C, G = sorter_of_the_dynamic_modeling(taus, dof)
    # print('MATRIZ DE INERCIA')
    # print(B)
    # print('MATRIZ DE CORIOLIS')
    # print(C)
    # print('PARES GRAVITACIONALES')
    # print(G)

    return B, C, G

def dynamic_modeling(table1, table2, M, j, direction_of_gravity):
    lag, dof = Lagrangian(table1, table2, M, j, direction_of_gravity)
    B, C, G = derivates_and_sorter(lag, dof)
    return B, C, G