#Kinematics
import numpy as np
from rkd.abc import *
from rkd.util import *
from rkd.transformations import *

def jacobiano(*matrices, type):
    #1. Que el usuario ingrese todas las matrices
    #2. Que el usuario ingrese los parametros DH en forma de arreglo y que la misma función se encargue de realizar las multiplicaciones de matrices correspondientes
    return