import numpy as np
import quaternion
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
CUBE = 2
PRECISION = 5

'''
Program służy do symulacji ewolucji równania kinematycznego
'''

def Rysowanie_wektorów(ax,u,v,w,color,letters):
    pocz_xyz = np.array([0, 0, 0])
    
    # Rysowanie wektorów
    Q = ax.quiver(pocz_xyz, pocz_xyz, pocz_xyz, u, v, w, color=color)

    # Etykiety
    for i, letter in enumerate(letters):
        ax.text(pocz_xyz[i] + u[i], pocz_xyz[i] + v[i], pocz_xyz[i] + w[i], f'{letter}')

    # Ustawienia osi i etykiet
    ax.set_xlim([-CUBE, CUBE])
    ax.set_ylim([-CUBE, CUBE])
    ax.set_zlim([-CUBE, CUBE])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return Q

def Quaternion_rot(v, q):
    # Zapis wektora jako kwaternion np [1,0,0] -> [0,1,0,0]
    v_as_quat = np.quaternion(0, v[0],v[1],v[2])

    # Tworzenie kwaternionu reprezentującego rotację
    q_conj = np.conjugate(q)
    v_rot_as_quat = q_conj * v_as_quat * q
    
    # Wynik też jest kwaternionem więc wracamy do normalnej postaci
    v_rot_as_quat = quaternion.as_float_array(v_rot_as_quat)
    v_rot = np.array([v_rot_as_quat[1],v_rot_as_quat[2],v_rot_as_quat[3]])
    return v_rot

def funkcja_dq_i_dw(wektor: np.array):
    q = np.quaternion(wektor[0],wektor[1],wektor[2],wektor[3])
    omega = np.array([wektor[4], wektor[5], wektor[6]])
    J = np.array([[2,0,0],
                  [0,3,0],
                  [0,0,4]])
    
    Jinv = np.linalg.inv(J)


    dw = np.dot(-Jinv,(np.cross(omega,np.dot(J , omega))))
    

    omega_q = np.quaternion(0,omega[0],omega[1],omega[2])
    dq = 0.5 * q * omega_q
    
    wynik = np.concatenate((np.array([dq.w,dq.x,dq.y,dq.z]),dw))
    return wynik



def Runge_Kutta_method(func, wektor:np.array, tn):
    h = 0.001
    k1 = func(wektor)
    k2 = func(wektor + h*(k1/2))
    k3 = func(wektor + h*(k2/2))
    k4 = func(wektor + h*k3)

    f_next = wektor + (h/6) * (k1 + 2 * k2 + 2 * k3 + k4)
    
    tn_next = tn + h
    return f_next, tn_next
 
def get_vecs(time):
    global  wektory_u, wektory_v, wektory_w
    u = wektory_u[time]
    v = wektory_v[time]
    w = wektory_w[time]
    # print(f"Wektory: {u}\t{v}\t{w}")
    pocz_xyz = np.array([0, 0, 0])
    return pocz_xyz, pocz_xyz, pocz_xyz, u, v, w

def update(time,ax):
    global quiver, wektory_u, wektory_v, wektory_w
    quiver.remove()
    colors = ['r','g','b']
    quiver = ax.quiver(*get_vecs(time), color = colors)

if __name__ == "__main__":
    np.set_printoptions(linewidth=1000)
    # Rysowanie wektorów
    fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))
    colors = ['r','g','b']
    
    # Ustawienia osi i etykiet
    ax.set_xlim([-CUBE, CUBE])
    ax.set_ylim([-CUBE, CUBE])
    ax.set_zlim([-CUBE, CUBE])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


    u = np.array([1,0,0])
    v = np.array([0,1,0])
    w = np.array([-1,0,0])
    
    wektory_u = []
    wektory_u.append(u)
    wektory_v = []
    wektory_v.append(v)
    wektory_w = []
    wektory_w.append(w)
    times = []
    quiver = ax.quiver(*get_vecs(0))

    t = 0
    q = np.quaternion(1,2,3,4)
    omega = np.array([0,1,0])
    

    lista_omeg = []   
    lista_kwaternionów = []
    lista_omeg.append(omega)
    lista_kwaternionów.append(q)


    while t < 5:
        wektor0_do_f = np.concatenate((omega,np.array([q.w,q.x,q.y,q.z])))
        wektor_next, t = Runge_Kutta_method(funkcja_dq_i_dw, wektor0_do_f, t) 
        #print(wektor_next)

        q = np.quaternion(wektor_next[0],wektor_next[1],wektor_next[2],wektor_next[3])
        norm = np.absolute(q)
        q = q / norm
        # print(q)

        omega = np.array([wektor_next[4], wektor_next[5], wektor_next[6]])
        #print(omega)
        
        times.append(t)
        urot = Quaternion_rot(u, q)
        wektory_u.append(urot)
        vrot = Quaternion_rot(v, q)
        wektory_v.append(vrot)
        wrot = Quaternion_rot(w, q)
        wektory_w.append(wrot)

    
    # for i in range(len(wektory_u)):
    #     print(f"\nCHWILA {round(times[i], 3)}: {np.round(wektory_u[i], decimals=PRECISION)}\t{np.round(wektory_v[i],decimals=PRECISION)}\t{np.round(wektory_w[i], decimals=PRECISION)}")
    
    # Utworzenie animacji   
    ani = FuncAnimation(fig, update,fargs = [ax], frames=100, interval = 1)
    plt.show()


    
