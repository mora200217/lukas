import numpy as np
import matplotlib.pyplot as plt
import math

# --- Inverse kinematics placeholder ---
def inverse_kinematics(x, y, l1, l2):
    """
    Compute joint angles for a 2R manipulator.
    Replace with your IK equations.
    Returns (theta1, theta2).
    """
    theta1 = 0.0
    theta2 = 0.0

    r2 = x**2 + y**2 
    C = (r2 - l1**2 - l2**2) / (l1 * l2)

    q2 = np.arctan2(C, np.sqrt(1- C**2))
    q1 = np.pi -np.arctan2(l1 + l2 * np.cos(q2), l2  * np.sin(q2)) - np.arctan2(x, y)




    return q1, q2


# --- Example usage ---
l1, l2 = 7.0, 7.0   # link lengths

# Trajectory: a semicircle in workspace

# --- Trayectoria en forma de trébol (flor de loto) ---
phi = np.linspace(0, 2*np.pi, 300)
a_0=1
n=6
a=a_0/n
rm=4
b=rm/(1+a)
r=b*(1+a*np.cos(n*(phi)))


x_traj = r * np.cos(phi) + 3
y_traj = r * np.sin(phi)

theta1_list = []
theta2_list = []

for x, y in zip(x_traj, y_traj):
    th1, th2 = inverse_kinematics(x, y, l1, l2)
    theta1_list.append(th1)
    theta2_list.append(th2)

# --- Plot ---
plt.figure(figsize=(8,4))
plt.plot(theta1_list, label=r'$\theta_1$')
plt.plot(theta2_list, label=r'$\theta_2$')
plt.xlabel("Step")
plt.ylabel("Angle (rad)")
plt.legend()
plt.title("2R Inverse Kinematics: Joint Angles")
plt.grid(True)
plt.show()


