import numpy as np
import sympy as sp

from control import lqr


def LQR_gains(Q_diag, R_diag):
    """
    Calculate the LQR gains for a given system.

    Parameters
    ----------
    Q_diag (list): Diagonal elements of the Q matrix for state weighting.
    R_diag (list): Diagonal elements of the R matrix for control weighting.

    Returns
    -------
    np.ndarray: The LQR gain matrix K.
    """
    # ======= NEW PARAMS OCT 24, 2024 ======= #
    Jr = 0.018    # Moment of inertia of wheels [kg⋅m²]
    Mr = 2.2      # Mass of wheel [kg] (BIG WHEEL)
    # Mr = 1.6      # Mass of wheel [kg] (SMALL WHEEL)
    Jpth = 1.46   # Moment of inertia of chassis pitch [kg⋅m²]
    Jpd = 0.041   # Moment of inertia of chassis yaw [kg⋅m²]
    Mp = 4-0.62   # Mass of chassis [kg]
    R = 0.0846    # Radius of wheels [m]
    D = 0.46      # Distance between wheels [m]
    L = 0.367     # Distance to center of gravity [m]
    g = 9.807     # Gravity [m/s²]

    # Define symbolic variable for angle
    theta = sp.Symbol('theta')

    # Create the state matrix using small angle approximations:
    # cos(theta) ≈ 1
    # sin(theta) ≈ theta
    # Create symbolic matrix A using sympy
    M_pitch = sp.Matrix([
        [-1, 0, 0, -1/(2*Mr), 1/(2*Mr)],
        [-1, -L*1, 0, 1/Mp, 0], 
        [0, L*theta, 1/Mp, 0, 0],
        [0, -1, L*theta/Jpth, -L*1/Jpth, 0],
        [-1, 0, 0, 0, -R**2/(2*Jr)]
    ])
    M_pitch_inv = M_pitch.inv()

    # Define symbolic variables for forces and control
    f_R, f_P, Cth = sp.symbols('f_R f_P Cth')
    b = sp.Matrix([
        -f_R/(2*Mr),
        -f_P/Mp,
        g,
        Cth/Jpth,
        -R*Cth/(2*Jr)
    ])

    # Calculate the state-space representation
    q = (M_pitch_inv * b).subs(theta**2,0)

    # Derive elements of the A and B matrices
    A23 = q[0].diff(theta).evalf()
    A43 = q[1].diff(theta).evalf()
    B21 = q[0].diff(Cth).evalf()
    B41 = q[1].diff(Cth).evalf()

    # Yaw dynamics
    M_yaw = sp.Matrix([
        [1,-D/(2*Jpd),0],
        [-2/D,-1/Mr,1/Mr],
        [-2/D,0,-R**2/Jr]
    ])
    M_yaw_inv = M_yaw.inv()

    # Define symbolic variables for yaw forces and control
    f_d, Cd = sp.symbols('f_d Cd')
    b = sp.Matrix([
        0,
        f_d/Mr,
        -Cd*R/Jr
    ])

    # Calculate the yaw state-space representation
    q = (M_yaw_inv * b)

    # Derive elements of the B matrix for yaw
    B62 = q[0].diff(Cd).evalf()

    # Assemble the A and B matrices
    A = np.array([
        [0,1,0,0,0,0],
        [0,0,A23,0,0,0],
        [0,0,0,1,0,0],
        [0,0,A43,0,0,0],
        [0,0,0,0,0,1],
        [0,0,0,0,0,0]
    ], dtype=float)

    B = np.array([
        [0,0],
        [B21,0],
        [0,0],
        [B41,0],
        [0,0],
        [0,B62]
    ], dtype=float)

    # Define the Q and R matrices for LQR
    Q = np.diag(Q_diag)
    R = np.diag(R_diag)

    # Calculate the LQR gain matrix K
    K, S, E = lqr(A, B, Q, R)
    return K