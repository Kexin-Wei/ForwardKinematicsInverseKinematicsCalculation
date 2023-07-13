import numpy as np
import matplotlib.pyplot as plt
import sympy as sp

sp.init_printing(use_unicode=True)


def dh_matrix(theta, d, a, alpha):
    """
    return the DH matrix for the given parameters
    Args:
        theta (radius): angle about previous z, from old x to new x
        d (m): offset along previous z to the common normal
        a (m): length of the common normal, if is a revolute joint, this is the radius about previous z
        alpha (radius): angle about common normal, from old z axis to new z axis
    """
    A = sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),                sp.cos(alpha),               d],
        [0,              0,                            0,                           1]])
    return A


if __name__ == "__main__":
    # Define dh table in theta, d, a, alpha order
    pvx, angle, nvz, n_angle = sp.symbols('pvx angle nvz n_angle')
    ndx1, ndy1, ndz1 = sp.symbols('ndx1 ndy1 ndz1')
    ndx2, ndy2, ndz2 = sp.symbols('ndx2 ndy2 ndz2')
    ndy3 = sp.symbols('ndy3')

    dh_table = sp.Matrix([
        [0, 0, 0, 0], # base
        [0, pvx, 0, 0],  # pivot
        [angle, 0, 0, sp.pi], # needle_linear
        [sp.pi/2, nvz, ndx1, sp.pi/2], # needle_base
        [n_angle, 0, ndy3, 0] # needle_tip 
        ])
    
    nJoint = dh_table.shape[0]
    T = sp.eye(4)
    for i in range(nJoint):
        A = dh_matrix(dh_table[i, 0], dh_table[i, 1], dh_table[i, 2], dh_table[i, 3])
        T = T * A
        print(f"A{i}:")
        sp.pprint(A)
    print("T:")
    sp.pprint(T)
    print("Done")
