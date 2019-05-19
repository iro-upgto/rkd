from rkd.abc import *
from rkd.dynamics_algorithms import *
from sympy import *
import numpy as np

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
print('**********MODELO DINAMICO**********')
print(dynamic_modeling(table1, table2, M, j, 'y'))