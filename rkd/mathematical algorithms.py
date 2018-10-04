#mathematical_algorithms
from numpy import *
import numpy as np
init_printing(use_latex=True)

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