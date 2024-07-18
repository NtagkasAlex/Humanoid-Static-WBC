import numpy as np
import matplotlib.pyplot as plt

def cubic_hermite(t, p0, p1, m0, m1):
    t2 = t ** 2
    t3 = t ** 3
    h00 = 2 * t3 - 3 * t2 + 1
    h10 = t3 - 2 * t2 + t
    h01 = -2 * t3 + 3 * t2
    h11 = t3 - t2

    return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1
def spline_func(p0, p1, m0, m1, T):
    return lambda t: cubic_hermite(t / T, p0, p1, T * m0, T * m1)


def plan_foot_trajectory(T_SSP, P0, P1, midpoint_height):
    Pm = np.array([(P0[0] + P1[0]) / 2, (P0[1] + P1[1]) / 2, midpoint_height])

    t0, t1 = 0, 0.5 * T_SSP
    t2 = T_SSP

    x = np.array([P0[0], Pm[0], P1[0]])
    y = np.array([P0[1], Pm[1], P1[1]])
    z = np.array([P0[2], Pm[2], P1[2]])

    dx0, dy0, dz0 = 0, 0, 0
    dx2, dy2, dz2 = 0, 0, 0

    dx1 = (x[2] - x[0]) / (t2 - t0)
    dy1 = (y[2] - y[0]) / (t2 - t0)
    dz1 = (z[2] - z[0]) / (t2 - t0)


    u_m=6/(8*t1)*(P1-P0)
    dx1= u_m[0]
    dy1 =u_m[1]
    dz1 =u_m[2]

    spline_x1 = spline_func(x[0], x[1], dx0, dx1, t1)
    spline_y1 = spline_func(y[0], y[1], dy0, dy1, t1)
    spline_z1 = spline_func(z[0], z[1], dz0, dz1, t1)

    spline_x2 = spline_func(x[1], x[2], dx1, dx2, t1)
    spline_y2 = spline_func(y[1], y[2], dy1, dy2, t1)
    spline_z2 = spline_func(z[1], z[2], dz1, dz2, t1)

    return (spline_x1, spline_y1, spline_z1), (spline_x2, spline_y2, spline_z2), T_SSP

def get_trajectory_point(t_query, splines1, splines2, T_SSP):
    spline_x1, spline_y1, spline_z1 = splines1
    spline_x2, spline_y2, spline_z2 = splines2
    midpoint_time = 0.5 * T_SSP

    if t_query <= midpoint_time:
        x_point = spline_x1(t_query)
        y_point = spline_y1(t_query)
        z_point = spline_z1(t_query)
    else:
        t_adjusted = t_query - midpoint_time
        x_point = spline_x2(t_adjusted)
        y_point = spline_y2(t_adjusted)
        z_point = spline_z2(t_adjusted)

    return np.array([x_point, y_point, z_point])

def com_trajectory(P0, P1, height, v0, v1, T):
    

    spline_x = spline_func(P0[0], P1[0], v0[0], v1[0], T)
    spline_y = spline_func(P0[1], P1[1], v0[1], v1[1], T)

    return spline_x, spline_y, height

def get_com_trajectory_point(t, spline_x, spline_y, height):
    x = spline_x(t)
    y = spline_y(t)
    z = height
    return np.array([x, y, z])


if __name__=="__main__":
    T_SSP = 0.2
    P0 = np.array([0, -0.06, 0.03])
    P1 = np.array([0.08, -0.06, 0.03])     
    midpoint_height = 0.1    

    splines_foot1, splines_foot2, T_SSP = plan_foot_trajectory(T_SSP, P0, P1, midpoint_height)

    t_query = T_SSP 
    point = get_trajectory_point(t_query, splines_foot1, splines_foot2, T_SSP)

    t_fine = np.linspace(0, T_SSP, 100)
    foot_trajectory_points = np.array([get_trajectory_point(t, splines_foot1, splines_foot2, T_SSP) for t in t_fine])
    x_fine = foot_trajectory_points[:, 0]
    y_fine = foot_trajectory_points[:, 1]
    z_fine = foot_trajectory_points[:, 2]

    dx_dt = np.gradient(x_fine, t_fine)
    dy_dt = np.gradient(y_fine, t_fine)
    dz_dt = np.gradient(z_fine, t_fine)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_fine, y_fine, z_fine, label='Foot Trajectory')
    ax.scatter([P0[0], P1[0]], [P0[1], P1[1]], [P0[2], P1[2]], color='red')
    ax.scatter(point[0], point[1], point[2], color='blue')
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


    fig, ax = plt.subplots(2, 1, figsize=(10, 8))
    ax[0].plot(t_fine, dx_dt, label='Velocity X')
    ax[0].legend()
    ax[0].grid(True)

    ax[1].plot(t_fine, dz_dt, label='Velocity Z')
    ax[1].legend()
    ax[1].grid(True)

    plt.tight_layout()
    plt.show()