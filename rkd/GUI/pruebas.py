from numpy import *
import numpy as np
from rkd.util import *

def rec2sph(x, y, z, deg = False):
	r = sqrt((x**2) + (y**2) + (z**2))
	ro = sqrt((x**2) + (y**2))

	if ((x > 0) and (y > 0)):
		fi = arctan2(y, x)
	if ((x < 0) and (y > 0)):
		fi = pi - arctan2(y, x)
	if ((x < 0) and (y < 0)):
		fi = pi + arctan2(y, x)
	if ((x > 0) and (y < 0)):
		fi = (2*pi) - arctan2(y, x)
	if z > 0:
		theta = arctan2(ro, z)
	if z < 0:
		theta = pi - arctan2(ro, z)
	if z == 0:
		theta = 0

	if deg:
		fi = rad2deg(fi)
		theta = rad2deg(theta)

	return r, theta, fi

def rec2cyl(x, y, z, deg = False):
	r = sqrt((x**2) + (y**2) + (z**2))
	ro = sqrt((x**2) + (y**2))

	if ((x > 0) and (y > 0)):
		fi = arctan2(y, x)
	if ((x < 0) and (y > 0)):
		fi = pi - arctan2(y, x)
	if ((x < 0) and (y < 0)):
		fi = pi + arctan2(y, x)
	if ((x > 0) and (y < 0)):
		fi = (2*pi) - arctan2(y, x)

	if deg:
		fi = rad2deg(fi)

	return ro, fi, z

def rec2pol(x, y, z, deg = False):
	ro = sqrt((x**2) + (y**2))
	if z > 0:
		theta = arctan2(ro, z)
	if z < 0:
		theta = pi - arctan2(ro, z)
	if z == 0:
		theta = 0
	if ((x > 0) and (y > 0)):
		fi = arctan2(y, x)
	if ((x < 0) and (y > 0)):
		fi = pi - arctan2(y, x)
	if ((x < 0) and (y < 0)):
		fi = pi + arctan2(y, x)
	if ((x > 0) and (y < 0)):
		fi = (2*pi) - arctan2(y, x)
	if deg:
		theta = rad2deg(theta)
		fi = rad2deg(fi)

	return ro, theta, fi

def sph2rec(r, theta, fi, deg = False):
	if deg:
		theta = deg2rad(theta)
		fi = deg2rad(fi)
	x = r*sin(theta)*cos(fi)
	y = r*sin(theta)*sin(fi)
	z = r*cos(theta)

	return x, y, z

if __name__=="__main__":
    
	# Para esfericas a rectangulares 
	# 22.360679774997898, -26.565051177078, 0.0

	# Para rectangulares a esfericas
	#20, -10, 0

    print("RESULTADO: \n", sph2rec(22.360679774997898, -26.565051177078, 0.0, True))