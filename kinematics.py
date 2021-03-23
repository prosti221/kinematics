from sympy import *
import numpy as np
import math

L1 = 100.9
L2 = 222.1
L3 = 136.2

def forward(t_1, t_2, t_3):
    t_1 = np.radians(t_1)
    t_2 = np.radians(t_2)
    t_3 = np.radians(t_3)
    
    A1 = Matrix([[cos(t_1), 0, sin(t_1), 0], 
                 [sin(t_1), 0, -cos(t_1), 0], 
                 [0, 1, 0, L1], 
                 [0, 0, 0, 1]])



    A2 = Matrix([[cos(t_2), -sin(t_2), 0, L2 * cos(t_2)], 
                 [sin(t_2), cos(t_2), 0, L2 * sin(t_2)], 
                 [0, 0, 1, 0], 
                 [0, 0, 0, 1]])

    A3 = Matrix([[cos(t_3), -sin(t_3), 0, L3 * cos(t_3)], 
                 [sin(t_3), cos(t_3), 0, L3 * sin(t_3)], 
                 [0, 0, 1, 0], 
                 [0, 0, 0, 1]])

    T = A1 * A2 * A3
    return(T.col(3)[0:3])

def inverse(x, y, z):
    theta_1 = 0
    theta_2 = 0
    theta_3 = 0

    theta_1 = atan2(y, x)
    theta_1 = theta_1
    r = sqrt(x**2 + y**2)
    s = z - L1
    c = sqrt(r**2 + s**2)
    temp1 = L3**2 - L2**2 - c**2
    temp2 = -2*L2*c
    phi_1 = acos( (L3**2 - L2**2 - c**2) / (-2*L2*c))
    phi_2 = atan2(s, r)
    theta_2 = phi_2 - phi_1
    phi_3 = acos( (c**2 - L2**2 - L3**2) / (-2*L2*L3))
    theta_3 = np.pi - phi_3
    return [(math.degrees(theta_1)), (math.degrees(theta_2)), (math.degrees(theta_3))]

def jacobian(t1, t2, t3, v1, v2, v3):


def main():
    coord = [0, -323.9033, 176.6988]
    inv = inverse(*coord)
    forw = forward(inv[0], inv[1], inv[2])
    
    print("Coordinates: x:%f, y:%f, z:%f" % (coord[0], coord[1], coord[2]))
    print("Inverse kinematics: theta1:%f, theta2:%f, theta3:%f" % (inv[0], inv[1], inv[2]))
    print("Forward kinematics: x:%f, y:%f, z:%f" % (forw[0], forw[1], forw[2]))  
    
    result = [False, False, False]
    result = [True for i in range(3) if abs(float(coord[i]) - float(forw[i])) < 0.00001]
    print(result)

main()
