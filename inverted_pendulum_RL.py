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
Q = np.diag([1.0, 1.0, 1.0, 0.0])  # state cost matrix
R = np.diag([0.01])  # input cost matrix

delta_t = 0.1  # time tick [s]
sim_time = 5.0  # simulation time [s]

show_animation = True

x0 = np.array([
    [0.0],
    [0.0],
    [0.3],
    [0.0]
])



def main():
    x = np.copy(x0)
    time = 0.0
    K = min_cost(maxiter=2000, learning_rate=0.00035)
    while time < 20:
        time += delta_t
        u = -K@x
        # simulate inverted pendulum cart
        x = simulation(x, u)

        if show_animation:
            plt.clf()
            px = float(x[0, 0])
            theta = float(x[2, 0])
            plot_cart(px, theta)
            plt.xlim([-5.0, 2.0])
            plt.pause(0.001)

    print("Finish")
    print(f"x={float(x[0, 0]):.2f} [m] , theta={math.degrees(x[2, 0]):.2f} [deg]")
    print(K)
    print(cost_function(u))
    if show_animation:
        plt.show()


def simulation(x, u):
    A, B = get_model_matrix()
    x = A @ x + B @ u
    return x

def cost_function(K_r):
    cost = 0.0
    t = 0.0
    x = np.copy(x0)
    while t < sim_time:
        u = -K.dot(x)
        cost += np.dot(x.T, np.dot(Q, x)) + np.dot(u.T, np.dot(R, u))
        x = simulation(x, u)
        t += delta_t
    return cost

def min_cost(maxiter, learning_rate):
    global K
    grad = np.zeros_like(K, dtype=np.float64)
    epsilon = 1e-7  # small increment
    for _ in range(maxiter):
        # Compute the gradient of the cost function
        for i in range(K.shape[0]):
            for j in range(K.shape[1]):
                K[i, j] += epsilon
                cost_plus = cost_function(K_r=K) # 转换为标量值
                K[i, j] -= 2 * epsilon
                cost_minus = cost_function(K_r=K)  # 转换为标量值
                K[i, j] += epsilon
                grad[i, j] = np.clip((cost_plus - cost_minus) / (2 * epsilon),-1e3, 1e3)
        print(grad,K)
        K = K - learning_rate * grad
    return K

# def lqr_control(x):
#     global K
#     K = min_cost(maxiter=2000, learning_rate=0.001)
#     print(K)
#     u = -K @ x
#     A, B = get_model_matrix()
#     start = time.time()
#     elapsed_time = time.time() - start
#     return u


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
    return np.array(a).reshape(-1)


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

    wx = np.copy(ox) + bx[-1]
    wy = np.copy(oy) + by[-1]

    plt.plot(flatten(cx), flatten(cy), "-b")
    plt.plot(flatten(bx), flatten(by), "-k")
    plt.plot(flatten(rwx), flatten(rwy), "-k")
    plt.plot(flatten(lwx), flatten(lwy), "-k")
    plt.plot(flatten(wx), flatten(wy), "-k")
    plt.title(f"x: {xt:.2f} , theta: {math.degrees(theta):.2f}")

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    plt.axis("equal")


if __name__ == '__main__':
    K = np.array([[-4,-8,99,40]])
    main()