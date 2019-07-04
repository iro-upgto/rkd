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
        DH parameter
    d : int, float or symbol
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
    '''
    Function for obtain the matrices with DH parameters, then we can use these matrices to obtain "potential energy"
    '''
    Ts = []
    H = eye(4)
    for k in args:
        Ts.append(Dh(k[0], k[1], k[2], k[3]))
    for i in range(len(args)):
        H = simplify(H*Ts[i])

    return H

def potential_energy(RT, size_linear_velocity, size_table2, direction_of_gravity = 'y'):
    '''
    :param RT: List with the all DH parameters and this list has that parameter in order
    :param size_linear_velocity: Number of values in the list of linear velocity
    :param size_table2: Also it is the DOF (degrees of freedom)
    :param direction_of_gravity: Direction of gravity with respect to an global axis, i.e., X, Y or Z
    :return: List with the parameters of the potential energy

    The work of this function is obtain the potential energy of automatic way with help of the homogeneous matrices
    '''

    # Creating symbolic variables, this variables are required for the potential energy
    m = []
    for i in range(size_table2):
        m.append(eval('m'+str(int("".join(str(i+1))))))

    # Creating the list that will get all the matrices of the DH parameters
    u, U, H, Hr = [], [], [], []
    for i in range(len(RT)):
            H.append(simplify(DH(*RT[i])))
            Hr.append(nsimplify(DH(*RT[i]), tolerance=1e-5, rational=True))
    # Show values of the list with the matrices of the DH parameters
    print('Matrices T con parámetros DH')
    print(Hr)

    # Obtaining equations of the potential energy of the matrices of the DH parameters
    for i in range(len(RT)):
        if direction_of_gravity == 'x' or direction_of_gravity == 'X':
            u.append(simplify(H[i][:1,3]))
        elif direction_of_gravity == 'y' or direction_of_gravity == 'Y':
            u.append(H[i][1:2,3])
        elif direction_of_gravity == 'z' or direction_of_gravity == 'Z':
            u.append(H[i][2:3,3])
    # Creating the list with the equation of the potential energy
    for i in range(size_linear_velocity):
        U.append((m[i]*g)*u[i])
    return simplify(np.sum(U[:], axis = 0)) # We need add all the variables or equations of the potential energy

def check_params(r1,r2,r3,r4,r5,r6):
    '''
    This function just check that we do not haven an empty variables and we can work with variables with values
    And we obtain a list with variables with values type string
    '''
    R = []
    for r in [r1,r2,r3,r4,r5,r6]:
        if r != '':
            R.append(eval(r))
    return R

def Lagrangian(params_SN_SM = ('r1','r2','r3','r4','r5','r6'), params_CN_SM = ('r1','r2','r3','r4','r5','r6'), direction_of_gravity = 'y'):
    '''
    :param params_SN_SM: Tupla with the DH parameters but without considering the mass systems, i.e, this parameters are
     with full distances between cinematic pair, for example: l1, l2, l3, and so on.
    :param params_CN_SM: Tupla with the DH parameters but considering the mass systems, i.e, this parameters are with
     the distances to mass center, for example: lc1, lc2, lc3, and so on.
    :param direction_of_gravity: Enter the direction of gravity respect to a global axis
    :return: The equation of lagrangian
    '''
    #*******************************************************************************************************************
    #          TABLE 1 (DH parameters without considering the mass systems)
    #*******************************************************************************************************************
    t1r1, t1r2, t1r3, t1r4, t1r5, t1r6 = params_SN_SM[0], params_SN_SM[1], params_SN_SM[2], params_SN_SM[3], params_SN_SM[4], params_SN_SM[5]
    #*******************************************************************************************************************
    #          TABLE 2 (DH parameters considering the mass systems)
    #*******************************************************************************************************************
    t2r1, t2r2, t2r3, t2r4, t2r5, t2r6 = params_CN_SM[0], params_CN_SM[1], params_CN_SM[2], params_CN_SM[3], params_CN_SM[4], params_CN_SM[5]
    #*******************************************************************************************************************
    Table1 = check_params(t1r1, t1r2, t1r3, t1r4, t1r5, t1r6)
    Table2 = check_params(t2r1, t2r2, t2r3, t2r4, t2r5, t2r6)
    R1 = len(Table1)
    R2 = len(Table2)
    #*******************************************************************************************************************
    #       JACOBIANO APPLIED IN A GENERAL MANNER
    #*******************************************************************************************************************
    JT, JTr = [], []
    # The table2 will our reference and that's why we required control the table1
    if R1 >= R2:
        Table1 = Table1[0:R2-1]
        R1 = R2-1
    # If we have a only parameter in the table2, then we do not required the table1
    if  R2 == 1:
        R, RT = [], []
        R.append(Table2[0])
        JT.append(jac(*R))
        JTr.append(jac(*R))
        RT.append([Table2[0]])
    # But if we have more parameters in our table2, then if we required the table1 and we required other method
    elif R2 > 1:
        # We will obtain a list (RT) that will contain all the parameters, i.e., we will have a single table with DH
        # parameters
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
        # From the list RT let's do the calculation of all the necessary Jacobian and they will be in the list JT
        for i in range(len(RT)):
            JT.append(jac(*RT[i]))
            JTr.append(nsimplify(jac(*RT[i]), tolerance=1e-5, rational=True))
    print('Matrices de Jacobianos')
    print(JTr)
    # Creating the symbolic variables of the derivative of the variable q
    qp = []
    for i in range(R2):
        qp.append(eval('q'+str(int("".join(str(i+1))))+'p'))
    # Operation of multiplication of the Jacobian by the variables qp, to follow the method of the dynamic modeling.
    Jp = []
    print(len(JT))
    for i in range(len(JT)):
        if i == 0: Jp.append(JT[0][:,0]*qp[0])
        if i > 0:
            for j in range(i+1):
                Jp.append(JT[i][:,j]*qp[j])
    # As previous method separates all our matrices by columns and adds terms for each column, so we need the columns
    #  to return to their respective matrices
    JP = []
    JP.append(Jp[0])
    i = 1
    l, k, z, a = 0, 2, 2, 1
    for j in range(1,len(JT)):
        JP.append(Matrix(np.sum(Jp[i:j+k], axis = 0)))
        i += j+a
        k += z
        z += 1
    # Creating symbolic variables and then we can use for the kinetic energy
    m, j = [], []
    for i in range(R2):
        m.append(eval('m'+str(int("".join(str(i+1))))))
        j.append(eval('J'+str(int("".join(str(i+1))))))

    #*******************************************************************************************************************
    #       OBTAINING LINEAR AND ANGULAR SPEEDS
    #*******************************************************************************************************************
    v, vr, w, wr = [], [], [], []
    for i in range(len(JP)):
        v.append(simplify(JP[i][:3,0:i+1]))
        w.append(simplify(JP[i][3:,0:i+1]))
        vr.append(nsimplify(JP[i][:3, 0:i + 1], tolerance=1e-5, rational=True))
        wr.append(nsimplify(JP[i][3:, 0:i + 1], tolerance=1e-5, rational=True))
    print('Velocidades lineales')
    print(vr)
    print('Velocidades ángulares')
    print(wr)
    #*******************************************************************************************************************
    #           KINEMATIC ENERGY
    #*******************************************************************************************************************
    k = []
    for i in range(len(v)):
        k.append(simplify((0.5*m[i]*v[i].T*v[i])+(0.5*j[i]*w[i].T*w[i])))
    K = simplify(np.sum(k[:], axis = 0))

    #*******************************************************************************************************************
    #          POTENTIAL ENERGY
    #*******************************************************************************************************************
    U = potential_energy(RT, len(v), R2, direction_of_gravity)

    print('Energía cinética')
    print(nsimplify(K, tolerance=1e-5, rational=True))
    print('Energía potencial')
    print(nsimplify(U, tolerance=1e-5, rational=True))

    return simplify(K-U), len(JT)

def check_lists(p1, p2, p3, p4, p5, p6):
    '''
    This function just check that we do not haven an empty variables and we can work with variables with values
    And we obtain a list with variables with values type list
    '''
    R = []
    for r in [p1, p2, p3, p4, p5, p6]:
        if r != []:
            R.append(r)
    return R

def sorter_of_the_dynamic_modeling(tau_n, dof):
    '''
    :param tau_n: The equation with the values obtained of the partial derivatives applied to the equation of Lagrangian
    :param dof: Number of Degrees of freedom
    :return: Sorter of matrices/vector to the dynamic modeling and they are B, C and G (this is a vector)
    '''
    q, qp, qpp = [], [], []
    for i in range(dof):
        q.append(eval('q'+str(int("".join(str(i+1))))))
        qp.append(eval('q'+str(int("".join(str(i+1))))+'p'))
        qpp.append(eval('q'+str(int("".join(str(i+1))))+'p'+'p'))

    #*******************************************************
    #              CHECK TAU VECTOR
    #*******************************************************
    q_pp, Gravity = [], []
    # In the case if we have one degree of freedom
    if dof == 1:
        qpp1, gravity = [], []
        qpp1.append(tau_n[0][0])
        # Check the equation of tau if match with some variable of interest
        for sumando in tau_n[0][0].args:
            for fc in sumando.args:
                if g == fc:
                    gravity.append(sumando)
        # Save in a list the equations of interest
        Gravity.append(gravity)
        r = []
        r.append(qpp1)
    # In the case if we have more than one degree of freedom
    elif dof > 1:
        for i in range(dof):
            qpp1, qpp2, qpp3, qpp4, qpp5, qpp6, gravity = [], [], [], [], [], [], []
            # Check the equation of tau if match with some variable of interest
            for sumando in tau_n[i][0].args:
                for fc in sumando.args:
                    if g == fc:
                        gravity.append(sumando)
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
            # Save in a list the equations of interest
            Gravity.append(simplify(np.sum(gravity[:], axis=0)))
            for r in [qpp1, qpp2, qpp3, qpp4, qpp5, qpp6]:
                if r != []:
                    q_pp.append(r)
        # In the variable q_pp we have many variables and they are in different list, therefore, we have to add them
        # and in that way have a single equation in a list
        r = []
        for i in range(len(q_pp)):
            r.append([np.sum(q_pp[i][:], axis = 0)])
    # As these variables are no longer necessary, now we are going to eliminate them
    i, res = 0, []
    while i < len(r):
        for j in range(dof):
            res.append(Matrix([simplify(r[i][0]/qpp[j])]))
            i += 1
    #*******************************************************
    #           OBTAINING THE "INERTIA MATRIX"
    #*******************************************************
    # Sorter of the inertia matrix
    B, k = ones(dof, dof), 0
    while k < len(r):
        for i in range(dof):
            for j in range(dof):
                B[i,j] = simplify(B[i,j]*res[k])
                k += 1

    #*******************************************************
    #           OBTAINING THE "CORIOLIS MATRIX"
    #*******************************************************
    # Application of the algorithm "Symbols of Christoffel"
    C, p = ones(dof**3, 1), 0
    sizeC,_ = C.shape
    while p < sizeC:
        for k in range(dof):
            for i in range(dof):
                for j in range(dof):
                    C[p,0] = C[p,0]*0.5*(B[i,j].diff(q[k]) + B[i,k].diff(q[j]) - B[j,k].diff(q[i]))*qp[k]
                    p += 1
    # Table required for the algorithm
    Cor, i = ones(dof**2,1), 0
    sizeCor,_ = Cor.shape
    if dof == 1: Cor[0,0] = simplify(C[0,0])
    if dof > 1:
        while i < sizeCor:
            Cor[i,0] = simplify(Cor[i,0]*C[i,0])
            for k in range(1,dof):
                Cor[i,0] = simplify(Cor[i,0]+C[i+(k*dof**2),0])
            i += 1
    # Sorter of the coriolis matrix
    C, k = ones(dof, dof), 0
    while k < len(Cor):
        for i in range(dof):
            for j in range(dof):
                C[i,j] = simplify(C[i,j]*Cor[k])
                k += 1
    # *******************************************************
    #           OBTAINING THE "VECTOR OF GRAVITATIONAL PAIRS"
    # *******************************************************
    G, k = ones(dof, 1), 0
    while k < len(Gravity):
        for i in range(dof):
            G[i,0] = simplify(G[i,0]*Gravity[k])
            if G[i,0] == 1:
                G[i,0] = 0
            k += 1

    return B, C, G

def derivates_and_sorter(lagrangian, dof):
    '''
    :param lagrangian: Equation obtained of the function Lagrangian
    :param dof: Number of degree of freedom
    :return: Matrices of inertia and coriolis, and vector of gravitational pairs
    '''
    diffq, diffqp, difft, q, qp, qpp, taus, taus_r = [], [], [], [], [], [], [], []
    for i in range(dof):
        q.append(eval('q'+str(int("".join(str(i+1))))))
        qp.append(eval('q'+str(int("".join(str(i+1))))+'p'))
        qpp.append(eval('q'+str(int("".join(str(i+1))))+'p'+'p'))
        #      Necessary derivates for dynamic modeling
        diffq.append(simplify(lagrangian.diff(q[i])))
        diffqp.append(simplify(lagrangian.diff(qp[i])))
        difft.append(simplify(diffqp[i].diff(t)))
        taus.append(simplify(difft[i]-diffq[i]))

    print('TAUS')
    print(taus)

    B, C, G = sorter_of_the_dynamic_modeling(taus, dof)

    return B, C, G

def dynamic_modeling(table1, table2, direction_of_gravity):
    '''
    :param table1: DH parameters without considering the mass systems
    :param table2: DH parameter considering the mass systems
    :param direction_of_gravity: Enter the direction of gravity respect to a global axis
    :return: Matrices of inertia and coriolis, and vector of gravitational pairs

    Principal Function
    '''
    lag, dof = Lagrangian(table1, table2, direction_of_gravity)
    print('Lagrangiano')
    print(nsimplify(lag, tolerance=1e-5, rational=True))
    B, C, G = derivates_and_sorter(lag, dof)
    return nsimplify(B, tolerance=1e-5, rational=True), nsimplify(C, tolerance=1e-5, rational=True), nsimplify(G, tolerance=1e-5, rational=True)