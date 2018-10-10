#mathematical_algorithms
from numpy import *
import numpy as np

def newton_raphson():
	k=1
	b=b.subs(vals).evalf()
	while True:
		x=((J.subs(X0).subs(vals)).inv()*b.subs(X0)).evalf()
		if x.norm()<eps: break
		for jj,ky  in enumerate(X0):
			X0[ky] += (x[jj]).evalf()
		k += 1
	return X0, x, k

def m_mult(*matrices):
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