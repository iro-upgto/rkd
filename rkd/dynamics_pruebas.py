from rkd.didactic.util import *
from rkd.didactic.core import *
from rkd.dynamics_algorithms import *
from sympy import *
import numpy as np

def check_params(r1,r2,r3,r4,r5,r6):
    R = []
    for r in [r1,r2,r3,r4,r5,r6]:
        if r != '':
            R.append(eval(r))
    return R

# NOTA: Meter los ángulos en radianes
# v1 = []
# v1 = '(lc1,0,0,q1)'
# v1.append(eval(v1))
# v2 = []
# v2 = '(l1,0,0,q1),(lc2,0,0,q2)'
# v2.append(eval(v2))

def Lagrangian(params_SN_SM = ('r1','r2','r3','r4','r5','r6'), params_CN_SM = ('r1','r2','r3','r4','r5','r6')):
    #*********************************************************************************************************************************************
    #       TABLE 1 (Parámetros DH sin considerar los sistemas de masas)
    #*********************************************************************************************************************************************
    t1r1, t1r2, t1r3, t1r4, t1r5, t1r6 = params_SN_SM[0], params_SN_SM[1], params_SN_SM[2], params_SN_SM[3], params_SN_SM[4], params_SN_SM[5]
    #*****************************
    #       TABLE 2 (Parámetros DH considerando los sistemas de masas)
    #*********************************************************************************************************************************************
    t2r1, t2r2, t2r3, t2r4, t2r5, t2r6 = params_CN_SM[0], params_CN_SM[1], params_CN_SM[2], params_CN_SM[3], params_CN_SM[4], params_CN_SM[5]
    #*********************************************************************************************************************************************
    Table1 = check_params(t1r1, t1r2, t1r3, t1r4, t1r5, t1r6)
    R1 = np.shape(Table1)[0]
    Table2 = check_params(t2r1, t2r2, t2r3, t2r4, t2r5, t2r6)
    R2 = np.shape(Table2)[0]
    if R1 == R2:
        Table1 = Table1[0:-1]
    if  R2 == 1:
        R = []
        R.append(Table2[0])
        r = jac(*R)
    elif R2 > 1:
        RT = []
        for j in range(R2, 0, -1):
            R = []
            k = j
            i = R1
            # for i in range(R1-1):
            i -= R2-k+1
            if i >= 0:
                for t in range(i):
                    R.append(Table1[t])
            R.append(Table2[j-1])
            RT.append(R)
        # r = jac(*JT)
        # R.append(Table2[-1])

        # r1 = Table1[0]
        # r2 = Table1[1]
        # r3 = Table1[2]
        # Table1 = tuple((r1,r2,r3))
        # Table1 = np.array(Table1)
        # Table1 = Table1[-1]

    # Table2 = np.array(check_params(t2r1, t2r2, t2r3, t2r4, t2r5, t2r6))
    # _,R2 = Table2.shape
    # for i in range(R1):
    # r = Table1[0]
    # R = Robot(r)
    # JT1 = R.J

    # R.append(v1)
    # R.append(w1)
    # R.append(v2)
    # R.append(w2)
    # R.append(v3)
    # R.append(w3)
    # R.append(v4)
    # R.append(w4)
    # R.append(v5)
    # R.append(w5)
    # R.append(v6)
    # R.append(w6)
    return RT

# J1 = Ji(r1)
# J2 = Ji(r2)
# J1 = J1[0]
# J2 = J2[1]
# Q1 = Matrix([q1p])
# Q12 = Matrix([q1p,q2p])
# J1 = simplify(J1*Q1)
# J2 = simplify(J2*Q12)
# v1 = J1[:3,0]
# v2 = J2[:3,0:1]
# w1 = J1[3:6,0]
# w2 = J2[3:6,0:1]
# print('J1 = ',J1)
# print('\n\nJ2 = ',J2)
# print('\n\nV1 = ',v1)
# print('\n\nV2 = ',v2)
# print('\n\nW1 = ',w1)
# print('\n\nW2 = ',w2)

a1 = 'l1,0,0,q1'
b1 = 'l2,0,0,q2'
c1 = 'l3,0,0,q3'
d1 = 'l4,0,0,q4'
e1 = ''
f1 = ''
# a1 = ''
# b1 = ''
# c1 = ''
# d1 = ''
# e1 = ''
# f1 = ''
a2 = 'lc1,0,0,q1'
b2 = 'lc2,0,0,q2'
c2 = 'lc3,0,0,q3'
d2 = 'lc4,0,0,q4'
e2 = ''
f2 = ''
# b2 = ''
# c2 = ''
# d2 = ''
# e2 = ''
# f2 = ''
table1 = (a1, b1, c1, d1, e1, f1)
table2 = (a2, b2, c2, d2, e2, f2)
print(Lagrangian(table1, table2))

# print(R2)
# print(R[0,0])
# print(R[0,1])
# print(R)
# r1=Robot(a)
# r2=Robot(b)
# r3 = Robot((c))
# r4 = Robot((d))
# r5 = Robot((e))
# r6 = Robot((f))
# r1, r2,r3, r4, r5, r6 = str(r1.J), str(r2.J), str(r3.J), str(r4.J), str(r5.J), str(r6.J)
# r3=Robot((l1,0,0,q1),(l2,0,0,q2),(lc3,0,0,q3))
# r4=Robot((l1,0,0,q1),(l2,0,0,q2),(l3,0,0,q3),(lc4,0,0,q4))
# r5=Robot((l1,0,0,q1),(l2,0,0,q2),(l3,0,0,q3),(l4,0,0,q4),(lc5,0,0,q5))
# r6=Robot((l1,0,0,q1),(l2,0,0,q2),(l3,0,0,q3),(l4,0,0,q4),(l5,0,0,q5),(lc6,0,0,q6))

# R = Lagrangian(r1,r2,'','','','')
# print(R)
# print(Lagrangian(r1,r2,r3,r4,r5,r6))