import pygame
import math

pygame.init()

WIDTH, HEIGHT = 800, 600
window = pygame.display.set_mode((WIDTH,HEIGHT))
pygame.display.set_caption("Gravitational slingshot effect")

# PLANET_MASS = 1.898e27
# SHIP_MASS = 333000
# G = 6.674e-11
# FPS = 60
# PLANET_RADIUS = 50
# OBJ_SIZE = 5
# VEL_SCALE = 100

PLANET_MASS = 100
SHIP_MASS = 5
G = 5
FPS = 60
PLANET_RADIUS = 50
OBJ_SIZE = 5
VEL_SCALE = 100

BG = pygame.transform.scale(pygame.image.load("Images/background.jpg"), (WIDTH,HEIGHT))
PLANET = pygame.transform.scale(pygame.image.load("Images/jupiter.png"), (PLANET_RADIUS*2,PLANET_RADIUS*2))
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

class Planet:
    def __init__(self, x, y, mass) -> None:
        self.x = x
        self.y = y
        self.mass = mass

    def draw(self):
        window.blit(PLANET, (self.x - PLANET_RADIUS, self.y - PLANET_RADIUS))




class Spacecraft:
    def __init__(self, x, y, vel_x, vel_y, mass):
        self.x = x
        self.y = y
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.mass = mass

    def move(self, planet=None):
        distance = math.sqrt((self.x - planet.x)**2 + (self.y - planet.y)**2)
        force = (G * self.mass * planet.mass) / distance**2
        acceleration = force / self.mass

        theta = math.atan2(planet.y - self.y , planet.x - self.x)

        acc_x = acceleration * math.cos(theta)
        acc_y = acceleration * math.sin(theta)

        self.vel_x += acc_x
        self.vel_y += acc_y

        self.x += self.vel_x
        self.y += self.vel_y

    def draw(self):
        pygame.draw.circle(window, RED, (int(self.x), int(self.y)), OBJ_SIZE)


def create_ship(Location, mouse):
    temp_x, temp_y = Location
    m_x, m_y = mouse
    vel_x = (m_x - temp_x)/VEL_SCALE
    vel_y = (m_y - temp_y)/VEL_SCALE

    obj = Spacecraft(temp_x, temp_y, vel_x, vel_y, SHIP_MASS)
    return obj


def main():
    running = True
    clock = pygame.time.Clock()

    objects = []
    temp_obj_pos = None

    Jupiter = Planet(WIDTH // 2, HEIGHT // 2, PLANET_MASS)

    while running:
        clock.tick(FPS)
        mouse_pos = pygame.mouse.get_pos()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                if temp_obj_pos:
                    
                    obj = create_ship(temp_obj_pos, mouse_pos) 
                    objects.append(obj)
                    temp_obj_pos = None
                else:
                    temp_obj_pos = mouse_pos 

        window.blit(BG, (0,0))




        if temp_obj_pos:
            pygame.draw.circle(window, RED, temp_obj_pos, OBJ_SIZE)
            pygame.draw.line(window, WHITE,temp_obj_pos, mouse_pos, 2)

        for obj in objects[:]: #Kopiujemy listę w  ten sposób żeby skończyć iterację
            obj.draw()
            obj.move(Jupiter)
            off_screen = obj.x < 0 or obj.x > WIDTH or obj.y < 0 or obj.y > HEIGHT
            collided = math.sqrt((obj.x - Jupiter.x)**2 + (obj.y - Jupiter.y)**2) <= PLANET_RADIUS
            if off_screen or collided:
                objects.remove(obj)
        
        Jupiter.draw()
 
        pygame.display.update()
    
    pygame.quit()

if __name__ == "__main__":
    main()




