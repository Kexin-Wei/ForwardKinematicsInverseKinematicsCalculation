import matplotlib.pyplot as plt
import sympy as sp

sp.init_printing(use_unicode=True)


def fromEulerToRotationMatrix(roll, pitch, yaw):
    matrix_roll = sp.Matrix([[1, 0, 0],
                            [0, sp.cos(roll), -sp.sin(roll)],
                            [0, sp.sin(roll), sp.cos(roll)]])
    matrix_pitch = sp.Matrix([[sp.cos(pitch), 0, sp.sin(pitch)],
                              [0, 1, 0],
                              [-sp.sin(pitch), 0, sp.cos(pitch)]])
    matrix_yaw = sp.Matrix([[sp.cos(yaw), -sp.sin(yaw), 0],
                            [sp.sin(yaw), sp.cos(yaw), 0],
                            [0, 0, 1]])
    return matrix_yaw * matrix_pitch * matrix_roll


def transformationMatrix(roll, pitch, yaw, x, y, z):
    rotation_matrix = fromEulerToRotationMatrix(roll, pitch, yaw)
    tranlation_vector = sp.Matrix([x, y, z])
    T = sp.Matrix([[rotation_matrix[0, 0], rotation_matrix[0, 1], rotation_matrix[0, 2], tranlation_vector[0]],
                   [rotation_matrix[1, 0], rotation_matrix[1, 1],
                       rotation_matrix[1, 2], tranlation_vector[1]],
                   [rotation_matrix[2, 0], rotation_matrix[2, 1],
                       rotation_matrix[2, 2], tranlation_vector[2]],
                   [0, 0, 0, 1]])
    return T


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
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),
         sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -
         sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),
         sp.cos(alpha),               d],
        [0,              0,                            0,                           1]])
    return A


def follow_dh_definition(dh_table):
    """Define dh table in theta, d, a, alpha order for forward kinematics
    Args:
        dh_table ( sp.Matrix ): an example is:
        
        d1, theta2, d3, theta4 = sp.symbols('d1 theta2 d3 theta4')
        l1, l2, l3, l4 = sp.symbols('l1 l2 l3 l4')

        dh_table = sp.Matrix([
            [0, d1, 0, 0],  # pivot
            [theta2, 0, 0, -sp.pi],  # needle_linear
            [sp.pi/2, d3, sp.sqrt(l2**2 + l3**2), sp.pi/2],  # needle_base
            [theta4, 0, l4, 0]  # needle_tip
        ])
    """
    nJoint = dh_table.shape[0]
    T = sp.eye(4)
    As = []

    for i in range(nJoint):
        A = dh_matrix(dh_table[i, 0], dh_table[i, 1],
                      dh_table[i, 2], dh_table[i, 3])
        T = T * A

        As.append(A)

        print(f"A{i}:")
        sp.pprint(A)

    print("T:")
    sp.pprint(T)
    print("Done")

    # inverse kinematics
    # define the end-effector position and orientation
    x, y, z = sp.symbols('x y z')
    a1, a2, a3 = sp.symbols('a1 a2 a3')
    Tend = transformationMatrix(a1, a2, a3, x, y, z)

    for i in range(nJoint):
        Tend = As[nJoint-1-i].inv() * Tend
    sp.simplify(Tend)
    print("Inverse kinematics:")
    sp.pprint(Tend)


if __name__ == "__main__":

    # follow_dh_definition()
    # define coordinate manually
    # forward kinematics
    print("Forward kinematics:")
    a, pvx = sp.symbols('a pvx')
    ndx1, ndy1, nvz, ndz1 = sp.symbols('ndx1 ndy1 nvz ndz1')
    na, ndx2, ndy2, ndz2 = sp.symbols('na ndx2 ndy2 ndz2')
    ndy3 = sp.symbols('ndy3')

    world_base = sp.Matrix([[-1, 0, 0, 0],
                            [0, 0, 1, 0],
                            [0, 1, 0, 0],
                            [0, 0, 0, 1]])
    base_pivot = transformationMatrix(0, a, 0, 0, pvx, 0)
    pivot_needle_linear = transformationMatrix(0, 0, 0, ndx1, ndy1, -nvz-ndz1)
    needle_linear_needle_base = transformationMatrix(
        na, 0, 0, -ndx2, ndy2, ndz2)
    needle_base_needle_tip = transformationMatrix(0, 0, 0, 0, -ndy3, 0)
    coordinates = [world_base, base_pivot, pivot_needle_linear,
                   needle_linear_needle_base, needle_base_needle_tip]
    for i in range(len(coordinates)):
        sp.pprint(coordinates[i])
        print("")
    T = sp.simplify(world_base * base_pivot * pivot_needle_linear *
                    needle_linear_needle_base * needle_base_needle_tip)

    sp.pprint(T)
    # inverse kinematics
    # define the end-effector position and orientation
    print("\n\nInverse kinematics:")
    x, y, z = sp.symbols('x y z')
    a1, a2, a3 = sp.symbols('a1 a2 a3')
    Tend = transformationMatrix(a1, a2, a3, x, y, z)
    # Tend = transformationMatrix(0, 0, 0, x, y, z)
    eqs = []
    for r in range(T.shape[0]):
        for c in range(T.shape[1]):
            eq = sp.Eq(Tend[r, c], T[r, c])
            eqs.append(eq)
    result = sp.solve(eqs, [a, pvx, nvz], dict=True)
    for i in range(len(result)):
        result_a = sp.simplify(result[i][a])
        result_pvx = sp.simplify(result[i][pvx])
        result_nvz = sp.simplify(result[i][nvz])
        print(f"\nSolution {i}:")
        print("a:")
        sp.pprint(result_a)
        print("pvx:")
        sp.pprint(result_pvx)
        print("nvz:")
        sp.pprint(result_nvz)
    print("Done")
