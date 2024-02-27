import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Przykładowe dane
x = [1, 2, 3, 4, 5]
y = [2, 3, 4, 5, 6]
z = [3, 4, 5, 6, 7]

# Tworzenie obiektu wykresu 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Rysowanie linii na wykresie
ax.plot(x, y, z, c='r', marker='o')

# Ustawienia osi i etykiet
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Wyświetlanie wykresu
plt.show()
