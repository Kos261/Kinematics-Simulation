import numpy as np
import quaternion
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
CUBE = 2


def Rysowanie_wektorów(ax,u,v,w,color,letters):
    pocz_x = np.array([0, 0, 0])
    pocz_y = np.array([0, 0, 0])
    pocz_z = np.array([0, 0, 0])
    # Rysowanie wektorów
    ax.quiver(pocz_x, pocz_y, pocz_z, u, v, w, color=color)

    # Etykiety
    for i, letter in enumerate(letters):
        ax.text(pocz_x[i] + u[i], pocz_y[i] + v[i], pocz_z[i] + w[i], f'{letter}')

    # Ustawienia osi i etykiet
    ax.set_xlim([-CUBE, CUBE])
    ax.set_ylim([-CUBE, CUBE])
    ax.set_zlim([-CUBE, CUBE])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    

def Vec_to_skew_matrix(v):
    return np.array([
        [0 , -v[2], v[1]],
        [v[2], 0 , -v[0]],
        [-v[1], v[0], 0 ]
    ])

def Quaternion_to_rotation_matrix(q):
    q = np.quaternion(q[0], q[1], q[2], q[3])
    return np.array([
        [1 - 2 * (q.y**2 + q.z**2), 2 * (q.x*q.y - q.z*q.w), 2 * (q.x*q.z + q.y*q.w)],
        [2 * (q.x*q.y + q.z*q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y*q.z - q.x*q.w)],
        [2 * (q.x*q.z - q.y*q.w), 2 * (q.y*q.z + q.x*q.w), 1 - 2 * (q.x**2 + q.y**2)]
    ])

def Rodriguez_formula(v, k, theta):
    #Normalizacja
    norm = np.linalg.norm(k, ord=2)
    k = k/norm
    vrot = v*np.cos(theta) + np.cross(k,v)*np.sin(theta) + k @ (k @ v)*(1- np.cos(theta))
    return vrot

def Rodriguez_parameters_from_q(q):
    if isinstance(q,np.quaternion):
        denom = 1 + q[0]
        return np.array([q[1]/denom, q[2]/denom, q[3]/denom])
    
def Rodriguez_parameters_from_vec(axis, theta):
    norm = np.linalg.norm(axis, ord=2)
    normalized_axis = axis / norm
    return normalized_axis * np.tan(theta / 2)

def Rotation_matrix():
    pass

def Euler_angles():
    pass

def Quaternion_rot(v, axis, theta):
    # Zapis wektora jako kwaternion np [1,0,0] -> [0,1,0,0]
    v_as_quat = np.quaternion(0, v[0],v[1],v[2])

    # Tworzenie kwaternionu reprezentującego rotację
    rot_q = np.quaternion(np.cos(theta / 2), np.sin(theta / 2) * axis[0], np.sin(theta / 2) * axis[1], np.sin(theta / 2) * axis[2])
    rot_q_conj = np.conjugate(rot_q)
    v_rot_as_quat = rot_q_conj * v_as_quat * rot_q
    
    # Wynik też jest kwaternionem więc wracamy do normalnej postaci
    v_rot_as_quat = quaternion.as_float_array(v_rot_as_quat)
    v_rot = np.array([v_rot_as_quat[1],v_rot_as_quat[2],v_rot_as_quat[3]])
    return v_rot

def funkcja(x):
    return 0.5 * x * np.array([0,0,0,1])

def Runge_Kutta_method(func,yn,tn):
    h = 0.01
    k1 = func(yn)
    k2 = func(yn + h*(k1/2))
    k3 = func(yn + h*(k2/2))
    k4 = func(yn + h*k3)

    yn_next = yn + (h/6) * (k1 + 2 * k2 + 2 * k3 + k4)
    tn_next = tn + h
    return yn_next, tn_next
 

if __name__ == "__main__":
    # Rysowanie wektorów
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    colors = ['r','g','b']

    u = np.array([1,0,0])
    v = np.array([0,1,0])
    w = np.array([0,0,1])


    Rysowanie_wektorów(ax,u,v,w, color=colors, letters = ['A1','A2','A3'])

    axis = np.array([0,0,1])
    axis = axis / np.linalg.norm(axis, ord=2)
    theta = np.pi/4
    urot = Quaternion_rot(u,axis,theta)
    vrot = Quaternion_rot(v,axis,theta)
    wrot = Quaternion_rot(w,axis,theta)
   

    Rysowanie_wektorów(ax,urot,vrot,wrot, color=colors, letters= ['B1','B2','B3'])

    plt.show()


    
