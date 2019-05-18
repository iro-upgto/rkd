from rkd.abc import *
from rkd.dynamics_algorithms import *
from sympy import *
import numpy as np

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
            R = []
            R.append(Table2[0])
            jaco = jac(*R)
            JT.append(jaco)
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

    return K - U, len(JT)

def dynamic_modeling(lagrangian, dof):
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

    # print('MATRIZ RESULTANTE FUNCION')
    # print(sorter_of_the_dynamic_modeling(taus, dof))
    # print('===============================')




    # NOTA: PROGRAMAR MATRIZ DE CORIOLIS

    return

# a1 = 'l1,pi/2,0,q1'
a1 = 'l1,0,0,q1'
b1 = 'l2,0,0,q2'
c1 = 'l3,0,0,q3'
d1 = 'l4,0,0,q4'
e1 = 'l5,0,0,q5'
# f1 = ''
# a1 = ''
# b1 = ''
# c1 = ''
# d1 = ''
# e1 = ''
f1 = ''
# a2 = 'lc1,pi/2,0,q1'
a2 = 'lc1,0,0,q1'
b2 = 'lc2,0,0,q2'
# c2 = 'lc3,0,0,q3'
# d2 = 'lc4,0,0,q4'
# e2 = ''
# f2 = ''
# b2 = ''
c2 = ''
d2 = ''
e2 = ''
f2 = ''

M1 = '10'
M2 = '20'
# M3 = '30'
# M4 = '40'
# M5 = ''
# M6 = ''
# M2 = ''
M3 = ''
M4 = ''
M5 = ''
M6 = ''
j1 = '10'
j2 = '20'
# j3 = '30'
# j4 = '40'
# j5 = ''
# j6 = ''
# j2 = ''
j3 = ''
j4 = ''
j5 = ''
j6 = ''
table1 = (a1, b1, c1, d1, e1, f1)
table2 = (a2, b2, c2, d2, e2, f2)
M = (M1, M2, M3, M4, M5, M6)
j = (j1, j2, j3, j4, j5, j6)
lag, dof = Lagrangian(table1, table2, M, j, 'y')
# print('***************************************************************')
# print(lag)
# print('***************************************************************')
# print(dof)

print(dynamic_modeling(lag, dof))