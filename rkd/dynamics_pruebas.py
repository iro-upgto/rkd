from rkd.didactic import *
from sympy import *

def Ji(r):
	J = []
	i = 1
	dof = r.dof
	while i <= dof:
	    J.append(r.J_i(i))
	    i += 1
	# m=[i for i,x in enumerate(J) if x!='']

	return J

# NOTA: Meter los ángulos en radianes
# v1 = []
# v1 = '(lc1,0,0,q1)'
# v1.append(eval(v1))
# v2 = []
# v2 = '(l1,0,0,q1),(lc2,0,0,q2)'
# v2.append(eval(v2))

def Lagrangian(r1,r2,r3,r4,r5,r6):
	R = []
	for r in [r1,r2,r3,r4,r5,r6]:
		if r != '':
			if r1 != '':
				J1 = Ji(r1)
				J1 = J1[0]
				Q1 = Matrix([q1p])
				J1 = simplify(J1*Q1)
				v1 = J1[:3,0]
				w1 = J1[3:6,0]
			if r2 != '':
				J2 = Ji(r2)
				J2 = J2[1]
				Q12 = Matrix([[q1p],[q2p]])
				J2 = simplify(J2*Q12)
				v2 = J2[:3,0:1]
				w2 = J2[3:6,0:1]
			if r3 != '':
				J3 = Ji(r3)
				J3 = J3[2]
				Q123 = Matrix([[q1p],[q2p],[q3p]])
				J3 = simplify(J3*Q123)
				v3 = J3[:3,0:2]
				w3 = J3[3:6,0:2]
			if r4 != '':
				J4 = Ji(r4)
				J4 = J4[3]
				Q1234 = Matrix([[q1p],[q2p],[q3p],[q4p]])
				J4 = simplify(J4*Q1234)
				v4 = J4[:3,0:3]
				w4 = J4[3:6,0:3]
			if r5 != '':
				J5 = Ji(r5)
				J5 = J5[4]
				Q12345 = Matrix([[q1p],[q2p],[q3p],[q4p],[q5p]])
				J5 = simplify(J5*Q12345)
				v5 = J5[:3,0:4]
				w5 = J5[3:6,0:4]
			if r6 != '':
				J6 = Ji(r6)
				J6 = J6[5]
				Q123456 = Matrix([[q1p],[q2p],[q3p],[q4p],[q5p],[q6p]])
				J6 = simplify(J6*Q123456)
				v6 = J6[:3,0:5]
				w6 = J6[3:6,0:5]
	R.append(v1)
	R.append(w1)
	R.append(v2)
	R.append(w2)
	# R.append(v3)
	# R.append(w3)
	# R.append(v4)
	# R.append(w4)
	# R.append(v5)
	# R.append(w5)
	# R.append(v6)
	# R.append(w6)
	return R

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

r1=Robot((lc1,0,0,q1))
r2=Robot((l1,0,0,q1),(lc2,0,0,q2))
# r3=Robot((l1,0,0,q1),(l2,0,0,q2),(lc3,0,0,q3))
# r4=Robot((l1,0,0,q1),(l2,0,0,q2),(l3,0,0,q3),(lc4,0,0,q4))
# r5=Robot((l1,0,0,q1),(l2,0,0,q2),(l3,0,0,q3),(l4,0,0,q4),(lc5,0,0,q5))
# r6=Robot((l1,0,0,q1),(l2,0,0,q2),(l3,0,0,q3),(l4,0,0,q4),(l5,0,0,q5),(lc6,0,0,q6))

print(Lagrangian(r1,r2,'','','',''))
# print(Lagrangian(r1,r2,r3,r4,r5,r6))