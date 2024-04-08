import pygame
import math
pygame.init()

WIDTH, HEIGHT = 1200, 700
WINDOW = pygame.display.set_mode((WIDTH,HEIGHT))
pygame.display.set_caption("Planet Simulation")

FONT = pygame.font.SysFont("comicsans", 16)
WHITE = (255,255,255)
BLACK = (0,0,0)
YELLOW = (255,255,0)
BLUE = (100,149,225)
RED = (188, 39, 50)
DARK_GREY = (80, 78, 81)
BG = pygame.transform.scale(pygame.image.load("Images/background.jpg"), (WIDTH,HEIGHT))

class Planet:
    AU = 149.6e6 * 1000
    G = 6.67428e-11
    SCALE = 250 / AU # 1 AU = 100 pixels
    TIMESTEP = 3600 * 24 # 1 day

    def __init__(self, x, y, radius, color, mass, sun = False, vel_x = 0, vel_y = 0) -> None:
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
        self.mass = mass
        self.vel_x = vel_x
        self.vel_y = vel_y

        self.sun = sun
        self.dist_to_sun = 0
        self.orbit = []

        

    def draw(self, win):
        x = self.x * self.SCALE + WIDTH / 2
        y = self.y * self.SCALE + HEIGHT / 2

        if len(self.orbit) > 2:
            updated_points = []
            for point in self.orbit:
                x, y = point
                x = x * self.SCALE + WIDTH/2
                y = y * self.SCALE + HEIGHT/2
                updated_points.append((x,y)) #Albo point
            pygame.draw.lines(win, self.color, False, updated_points, 2)

        pygame.draw.circle(win, self.color, (x,y), self.radius)

        if not self.sun:
            dist_text = FONT.render(f"{round(self.dist_to_sun/1000, 1)}km", 1, WHITE)
            win.blit(dist_text, (x-dist_text.get_width()/2, y-dist_text.get_width()/2))


    def attracction(self, other):
        other_x, other_y = other.x, other.y
        dist_x = other_x - self.x
        dist_y = other_y - self.y
        distance = math.sqrt((dist_x)**2 + (dist_y)**2)

        if other.sun:
            self.dist_to_sun = distance

        force = self.G * self.mass * other.mass / distance ** 2
        theta = math.atan2(dist_y, dist_x)
        force_x = math.cos(theta) * force
        force_y = math.sin(theta) * force

        return force_x, force_y
    

    def update_position(self, planets):
        total_fx = 0
        total_fy = 0
        for planet in planets:
            if self == planet:
                continue
            
            fx, fy = self.attracction(planet)
            total_fx += fx
            total_fy += fy

        self.vel_x += total_fx / self.mass * self.TIMESTEP
        self.vel_y += total_fy / self.mass * self.TIMESTEP
        self.x += self.vel_x * self.TIMESTEP
        self.y += self.vel_y * self.TIMESTEP

        self.orbit.append((self.x, self.y))

def main(): 
    running = True
    clock = pygame.time.Clock()

    Sun = Planet(0,0,30,YELLOW,1.98892 * 10 ** 30, sun=True)                                                                      #Velocity m/s                                 
    Earth = Planet(-1 * Planet.AU, 0, 16, BLUE, 5.9742 * 10 ** 24, vel_y = 29.783 * 1000)
    Mars = Planet(-1.524 * Planet.AU, 0, 12, RED, 6.39 * 10 ** 23, vel_y = 23.077 * 1000)
    Mercury = Planet(0.387 * Planet.AU, 0, 8, DARK_GREY, 3.30 * 10 ** 23, vel_y = -47.4 * 1000)
    Venus = Planet(0.723 * Planet.AU, 0, 14, WHITE, 4.8685 * 10 ** 24, vel_y = -35.02 * 1000)

    planets = [Sun, Earth, Mars, Mercury, Venus]

    while running:
        clock.tick(60)
        WINDOW.blit(BG, (0,0))
        

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for planet in planets:
            planet.update_position(planets)
            planet.draw(WINDOW)


        pygame.display.update() 


    pygame.quit()

if __name__ == "__main__":
    main()