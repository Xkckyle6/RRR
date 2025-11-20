# RRR.py

import math
import numpy as np


def fk(theta0, theta1, theta2, L0, L1, L2):
    """
    Forward kinematics for a 3-joint RRR arm.

    Joint layout:
      - theta0: base yaw about Z (up)
      - theta1: shoulder pitch about local Y
      - theta2: elbow pitch about local Y

    Link layout:
      - L0: vertical offset along global Z (base to shoulder)
      - L1: upper arm length
      - L2: forearm length

    Angle units: radians
    Returns: (x, y, z) end-effector position in base frame
    """

    r = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    z = L0 + L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)

    x = r * math.cos(theta0)
    y = r * math.sin(theta0)

    return x, y, z


def ik(x, y, z, L0, L1, L2, elbow_up=True):
    """
    Inverse kinematics for the same 3-joint RRR arm.

    Given:
      - desired end-effector position (x, y, z)
      - link lengths L0, L1, L2

    Solve for:
      - theta0: base yaw about Z
      - theta1: shoulder pitch
      - theta2: elbow pitch

    Angle units: radians

    Parameters:
      elbow_up (bool): 
          True  -> elbow-up configuration
          False -> elbow-down configuration

    Returns:
      (theta0, theta1, theta2)

    Raises:
      ValueError if the target is unreachable.
    """

    theta0 = math.atan2(y, x)

    r_xy = math.hypot(x, y)
    z_p = z - L0
    d = math.hypot(r_xy, z_p)

    cos_theta2 = (d**2 - L1**2 - L2**2) / (2.0 * L1 * L2)

    if cos_theta2 < -1.0 - 1e-9 or cos_theta2 > 1.0 + 1e-9:
        raise ValueError("Target is out of reach for this arm configuration.")

    cos_theta2 = max(-1.0, min(1.0, cos_theta2))

    theta2_mag = math.acos(cos_theta2)
    theta2 = -theta2_mag if elbow_up else theta2_mag

    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)

    theta1 = math.atan2(z_p, r_xy) - math.atan2(k2, k1)

    return theta0, theta1, theta2


def calc_joint_vel(theta0, theta1, theta2,
                               vx, vy, vz,
                               L0, L1, L2):
    """
    Compute joint velocities [theta0_dot, theta1_dot, theta2_dot]
    that produce a desired end-effector linear velocity [vx, vy, vz].

    Uses:
        v = J(theta) * q_dot   -->   q_dot = J^{-1}(theta) * v

    Inputs:
      - theta0, theta1, theta2 : current joint angles [rad]
      - vx, vy, vz             : desired EE velocity in base frame [m/s]
      - L0, L1, L2             : link lengths (L0 unused in Jacobian, but
                                 kept for consistent interface)

    Returns:
      (theta0_dot, theta1_dot, theta2_dot) [rad/s]

    Raises:
      ValueError if Jacobian is singular / ill-conditioned.
    """

    # Precompute some trig terms
    t0 = theta0
    t1 = theta1
    t2 = theta2

    r = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)

    # Derivatives of r and z wrt t1, t2
    dr_dt1 = -L1 * math.sin(t1) - L2 * math.sin(t1 + t2)
    dr_dt2 = -L2 * math.sin(t1 + t2)

    dz_dt1 = r  # = L1 cos t1 + L2 cos(t1 + t2)
    dz_dt2 = L2 * math.cos(t1 + t2)

    # Jacobian matrix J: v = J * q_dot
    # Rows: [vx; vy; vz], Cols: [t0_dot, t1_dot, t2_dot]
    J = np.array([
        [-r * math.sin(t0),   dr_dt1 * math.cos(t0),   dr_dt2 * math.cos(t0)],
        [ r * math.cos(t0),   dr_dt1 * math.sin(t0),   dr_dt2 * math.sin(t0)],
        [               0.0,             dz_dt1,                  dz_dt2    ]
    ])

    v = np.array([vx, vy, vz])

    # Try solving J q_dot = v
    # If Jacobian is singular/near-singular, fall back to least-squares
    try:
        q_dot = np.linalg.solve(J, v)
    except np.linalg.LinAlgError:
        # Pseudo-inverse solution (least squares)
        J_pinv = np.linalg.pinv(J)
        q_dot = J_pinv @ v

    theta0_dot, theta1_dot, theta2_dot = q_dot
    return theta0_dot, theta1_dot, theta2_dot


if __name__ == "__main__":
    # Demo: from desired EE velocity to joint velocities

    L0 = 0.5
    L1 = 0.3
    L2 = 0.2

    # Some current joint angles
    th0 = 0.3
    th1 = 0.4
    th2 = -0.2

    # Desired EE linear velocity in base frame
    vx, vy, vz = 0.05, 0.00, 0.02  # m/s

    w0, w1, w2 = calc_joint_vel(th0, th1, th2,
                                            vx, vy, vz,
                                            L0, L1, L2)

    print("Joint velocities for desired EE velocity (rad/s):")
    print(f"theta0_dot = {w0:.4f}")
    print(f"theta1_dot = {w1:.4f}")
    print(f"theta2_dot = {w2:.4f}")