from rkd.dynamics_algorithms import *

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

table1 = (a1, b1, c1, d1, e1, f1)
table2 = (a2, b2, c2, d2, e2, f2)
print('**********MODELO DINAMICO**********\n')
B, C, G = dynamic_modeling(table1, table2, 'y')
print('---------MATRIZ DE INERCIA-----------')
print(B)
print('\n--------MATRIZ DE CORIOLIS-----------')
print(C)
print('\n---VECTOR DE PARES GRAVITACIONALES---')
print(G)
