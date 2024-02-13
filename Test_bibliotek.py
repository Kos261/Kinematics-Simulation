import numpy as np
import quaternion


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

if __name__ == "__main__":
    np.set_printoptions(linewidth=1000)
    
    q = np.quaternion(1,2,3,4)
    
    
    omega = np.array([0.0,1.0,0.0])
    print(omega)

    wektor_do_f = np.concatenate((omega,np.array([q.w,q.x,q.y,q.z])))
    print(wektor_do_f)

    wynik = funkcja_dq_i_dw(wektor_do_f)
    print(wynik)