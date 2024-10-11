"""
Inverted Pendulum LQR control
Reference: https://github.com/AtsushiSakai/PythonRobotics
"""

import math
import time

import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv, eig

# Model parameters

l_bar = 2.0  # length of bar
M = 1.0  # [kg]
m = 0.3  # [kg]
g = 9.8  # [m/s^2]

nx = 4  # number of state
nu = 1  # number of input
Q = np.diag([1.0, 1.0, 1.0, 1.0])  # state cost matrix
R = np.diag([0.01])  # input cost matrix

delta_t = 0.05  # time tick [s]
sim_time = 10.0  # simulation time [s]

show_animation = True


def main():
    x0 = np.array([
        [0.0],
        [0.0],
        [0.0],
        [1.0]
    ])

    x = np.copy(x0)
    time = 0.0

    Log = []
    ball_positions = []  # List to store ball positions
    plt.gca().set_facecolor('white')

    while 10 > time:
        time += delta_t

        # calc control input
        u, K = lqr_control(x)

        # simulate inverted pendulum cart
        x = simulation(x, u)

        Log.append((time, x[0, 0], x[2, 0], float(u[0,0])))

        if show_animation:
            plt.clf()
            px = float(x[0, 0])
            theta = float(x[2, 0])
            plot_cart(px, theta)
            plt.xlim([-5.0, 2.0])

            # Calculate ball position
            ball_x = px + l_bar * math.sin(theta)
            ball_y = l_bar * math.cos(theta) + 0.7
            ball_positions.append((ball_x, ball_y))

            # Plot ball trajectory
            # Plot ball trajectory
            ball_positions_np = np.array(ball_positions)
            plt.scatter(ball_positions_np[:, 0], ball_positions_np[:, 1], color='c', s=10)


            plt.pause(0.001)

    print("Finish")
    print(f"x={float(x[0, 0]):.2f} [m] , theta={math.degrees(x[2, 0]):.2f} [deg]")
    print(K, u)
    print(cost_function(x, u))
    if show_animation:
        plt.show()

    # Plotting px and theta over time
    times = [entry[0] for entry in Log]
    px_values = [entry[1] for entry in Log]
    theta_values = [entry[2] for entry in Log]
    u_values = [entry[3] for entry in Log]

    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(times, px_values, label='px [m]')
    plt.xlabel('Time [s]')
    plt.ylabel('s [m]')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(times, [math.degrees(theta) for theta in theta_values], label='theta [deg]')
    plt.xlabel('Time [s]')
    plt.ylabel('q [deg]')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(times, u_values, label='u [N]')
    plt.xlabel('Time [s]')
    plt.ylabel('Control Input u [N]')
    plt.grid(True)
    plt.legend()

    plt.tight_layout(pad=0.7)  # Adjust the padding between and around subplots
    plt.show()


def simulation(x, u):
    A, B = get_model_matrix()
    x = A @ x + B @ u

    return x


def cost_function(x, u):
    cost = 0.0
    for t in np.arange(0, sim_time, delta_t):
        cost += np.dot(x.T, np.dot(Q, x)) + np.dot(u.T, np.dot(R, u))
        x = simulation(x, u)
    return cost


def solve_DARE(A, B, Q, R, maxiter=1500, eps=0.01):
    """
    Solve a discrete time_Algebraic Riccati equation (DARE)
    """
    P = Q

    for i in range(maxiter):
        Pn = A.T @ P @ A - A.T @ P @ B @ \
            inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
        if (abs(Pn - P)).max() < eps:
            break
        P = Pn
        #print(P)

    return Pn


def dlqr(A, B, Q, R):
    """
    Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    P = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = inv(B.T @ P @ B + R) @ (B.T @ P @ A)

    eigVals, eigVecs = eig(A - B @ K)
    return K, P, eigVals


def lqr_control(x):
    A, B = get_model_matrix()
    start = time.time()
    K, _, _ = dlqr(A, B, Q, R)
    u = -K @ x
    elapsed_time = time.time() - start
    # print(f"calc time:{elapsed_time:.6f} [sec]")

    return u, K


def get_numpy_array_from_matrix(x):
    """
    get build-in list from matrix
    """
    return np.array(x).flatten()


def get_model_matrix():
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, m * g / M, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, g * (M + m) / (l_bar * M), 0.0]
    ])
    A = np.eye(nx) + delta_t * A

    B = np.array([
        [0.0],
        [1.0 / M],
        [0.0],
        [1.0 / (l_bar * M)]
    ])
    B = delta_t * B

    return A, B


def flatten(a):
    return np.array(a).flatten()


def plot_cart(xt, theta):
    cart_w = 1.0
    cart_h = 0.5
    radius = 0.1

    cx = np.array([-cart_w / 2.0, cart_w / 2.0, cart_w /
                   2.0, -cart_w / 2.0, -cart_w / 2.0])
    cy = np.array([0.0, 0.0, cart_h, cart_h, 0.0])
    cy += radius * 2.0

    cx = cx + xt

    bx = np.array([0.0, l_bar * math.sin(-theta)])
    bx += xt
    by = np.array([cart_h, l_bar * math.cos(-theta) + cart_h])
    by += radius * 2.0

    angles = np.arange(0.0, math.pi * 2.0, math.radians(3.0))
    ox = np.array([radius * math.cos(a) for a in angles])
    oy = np.array([radius * math.sin(a) for a in angles])

    rwx = np.copy(ox) + cart_w / 4.0 + xt
    rwy = np.copy(oy) + radius
    lwx = np.copy(ox) - cart_w / 4.0 + xt
    lwy = np.copy(oy) + radius

    wx = np.copy(ox) + bx[1]
    wy = np.copy(oy) + by[1]

    plt.fill(flatten(cx), flatten(cy), 'royalblue', edgecolor='k')  # Fill color for the cart
    plt.plot(flatten(bx), flatten(by), "-k")
    plt.fill(flatten(rwx), flatten(rwy), "gray", edgecolor='k')  # Fill color for the right wheel
    plt.fill(flatten(lwx), flatten(lwy), "gray", edgecolor='k')  # Fill color for the left wheel
    plt.fill(flatten(wx), flatten(wy), "lightsteelblue", edgecolor='k')  # Fill color for the pendulum ball
    plt.title(f"x: {xt:.2f} , theta: {math.degrees(theta):.2f}")

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
    plt.axis("equal")


if __name__ == '__main__':
    main()
