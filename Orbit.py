import numpy as np
import quaternion
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

MI = 3.9e+5
H = 0.5
CUBE = 15000

def Runge_Kutta_method(func, wektor:np.array, tn):
    k1 = func(wektor)
    k2 = func(wektor + H*(k1/2))
    k3 = func(wektor + H*(k2/2))
    k4 = func(wektor + H*k3)

    f_next = wektor + (H/6) * (k1 + 2 * k2 + 2 * k3 + k4)
    
    tn_next = tn + H
    return f_next, tn_next

def shortcut_a(x):
    wynik = (- MI ) / ((np.sqrt(x[0]**2 + x[1]**2 + x[2]**2))**3)
    return wynik

def funkcja(x: np.array):
    a = shortcut_a(x)
    dx = np.array([x[3], x[4], x[5], x[0]*a, x[1]*a, x[2]*a])
    return dx


if __name__ =='__main__':
    x0 = 7000 #km
    y0 = 0
    z0 = 0
    vx0 = 0
    vy0 = 8 #km/s
    vz0 = 0
    vec_next = np.array([x0,y0,z0, vx0,vy0,vz0])
    t = 0 
    How_long = 1000

    x_coords_in_time = []
    y_coords_in_time = []
    z_coords_in_time = []


    while t < How_long:
        vec_next, t = Runge_Kutta_method(funkcja,vec_next,t)
        x_coords_in_time.append(vec_next[0])
        y_coords_in_time.append(vec_next[1])
        z_coords_in_time.append(vec_next[2])


    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.set_xlim([-CUBE, CUBE])
    ax.set_ylim([-CUBE, CUBE])
    ax.set_zlim([-CUBE, CUBE])
    # times = range(int(How_long * 1/H))
    # plt.plot(times, x_coords_in_time, times, y_coords_in_time, times, z_coords_in_time)
    for i in range(int(How_long * 1/H)):
        ax.plot(x_coords_in_time[i],y_coords_in_time[i],z_coords_in_time[i],c='r')
    plt.show()