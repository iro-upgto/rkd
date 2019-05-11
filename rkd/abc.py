from sympy import symbols
from sympy.physics.mechanics import dynamicsymbols

g,t = symbols('g t', real=True)
t1,t2,t3,t4,t5,t6 = symbols('\\theta_1:7', real=True)
q1,q2,q3,q4,q5,q6 = symbols('q_1:7', real=True)
q1p,q2p,q3p,q4p,q5p,q6p = dynamicsymbols('\\dot{q}_1:7')
q1pp,q2pp,q3pp,q4pp,q5pp,q6pp = symbols('\\ddot{q}_1:7')
l1,l2,l3,l4,l5,l6 = symbols('l_1:7', real=True)
lc1,lc2,lc3,lc4,lc5,lc6 = symbols('l_c1:7', real=True)
d1,d2,d3,d4,d5,d6 = symbols('d_1:7', real=True)
m1,m2,m3,m4,m5,m6 = symbols('m_1:7', real=True)
J1,J2,J3,J4,J5,J6 = symbols('J_1:7', real=True)
a1,a2,a3,a4,a5,a6 = symbols('a_1:7', real=True)
x0,x1,x2,x3,x4,x5,x6 = symbols("x_0:7", real=True)
y0,y1,y2,y3,y4,y5,y6 = symbols("y_0:7", real=True)
z0,z1,z2,z3,z4,z5,z6 = symbols("z_0:7", real=True)

# Some common greek letters 
alpha,beta,gamma,delta = symbols("alpha,beta,gamma,delta", real=True)
epsilon,zeta,eta,theta = symbols("epsilon,zeta,eta,theta", real=True)
iota,kappa,mu,nu = symbols("iota,kappa,mu,nu", real=True)
xi,omicron,rho,sigma = symbols("xi,omicron,rho,sigma", real=True)
tau,upsilon,phi,chi = symbols("tau,upsilon,phi,chi", real=True)
psi,omega = symbols("psi,omega", real=True)

# ~ del pi # Delete "pi" symbolic variable -> conflict with pi number