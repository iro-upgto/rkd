"""
Universidad Politécnica de Guanajuato
Departamento de Ingeniería Robótica, (c) 2019

This module has been designed for academic purposes, using SymPy as base library. 
It's easy to check that SymPy is slower than NumPy specially in matrix algebra, 
however SymPy is more convenient to use as didactic tool due to the given facilities 
as the symbolic manipulation, calculation of partial and ordinary derivatives, 
matricial multiplication using asterisk symbol, "init_printing" function and so on.
"""

from sympy import solve, symbols, init_printing, pi, simplify
from sympy.matrices import Matrix, eye, zeros, ones
from rkd.abc import * # To use common symbolic variables
from .core import * 
from .plotting import * 
from .transformations import * 
from .util import *
# ~ from .ws import * # not yet ready
init_printing() # Get "pretty print"
