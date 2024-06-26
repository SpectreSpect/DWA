import pygame
import sys
import numpy as np
from typing import List
from dynamic_window_approach import *
import copy
import time

class Drawable():
    def update(self):
        pass


    def draw(self, screen):
        pass

class Obstacle(Drawable):
    def __init__(self, position: np.ndarray, radius: float, color: tuple):
        self.position = position
        self.radius = radius
        self.color = color
    
    def get_obstacles(self):
        return [self.position]
    
    def draw(self, screen):
        pygame.draw.circle(screen, self.color, self.position, self.radius)
        

class Line(Obstacle):
    def __init__(self, points: np.ndarray, color: tuple):
        super().__init__(points[0], points[0], color)
        self.points = points
    
    def get_obstacles(self):
        return self.points
    
    def draw(self, screen):
        draw_line(screen, self, self.color)


class Robot(Drawable):
    def __init__(self, 
                 position: np.ndarray, 
                 min_velocity: float, 
                 max_velocity: float,
                 speed: float,
                 omega_speed: float,
                 distance_to_inter_move: float,
                 radius: int, 
                 color: tuple, 
                 velocity: float = 0,
                 angle: float = 0,
                 omega: float = 0):
        self.position = position # позиция
        self.min_velocity = min_velocity # минимальная скорость движения
        self.max_velocity = max_velocity # максимальная скорость движения
        self.speed = speed # скорость движения
        self.omega_speed = omega_speed # угловая скорость
        self.distance_to_linear_move = distance_to_inter_move # дистанция до движения с интерполяцией
        self.inter_move_factor = 0
        self.radius = radius # радиус робота
        self.color = color # цвет робота
        self.velocity = velocity # скорость (метр./сек.)
        self.angle = angle # угол робота (рад.)
        self.omega = omega # скорость изменения угла робота, производная угла (рад./сек.)
    
    
    def move_to_goal(self, goal: np.ndarray, obstacles: List[Obstacle], dt: float):
        x = [self.position[0], self.position[1], self.angle, self.velocity, self.omega]
        
        obstacle_positions = []
        for obstacle in obstacles:
            obstacle_positions += list(obstacle.get_obstacles())
        
        obstacle_positions = np.array(obstacle_positions)

        config.ob = obstacle_positions

        #config.dt = 0.2
        u, predicted_trajectory = dwa_control(x, config, goal, obstacle_positions)
        x = motion(x, u, dt)
        _, _, _, self.velocity, self.omega = x
    

    def update_robot_state(self, goal: np.ndarray, dt: float):
        goal_diff = goal - self.position
        goal_distance = np.linalg.norm(goal_diff)
        if goal_distance > self.distance_to_linear_move:
            # DWA
            delta_angle = self.omega * self.omega_speed * dt
            self.angle += delta_angle

            speed = self.velocity * self.speed
            speed = min(max(self.min_velocity, speed), self.max_velocity)

            R = np.array([[np.cos(self.angle), -np.sin(self.angle)],
                        [np.sin(self.angle), np.cos(self.angle)]])
            vel = np.dot(R, np.array([1, 0]))
            vel *= speed

            self.position += vel * dt

            self.inter_move_factor = np.sqrt(goal_distance) / speed
        else:
            # Прямолинейное движение
            goal_direction = goal_diff / goal_distance
            speed = np.sqrt(goal_distance) / self.inter_move_factor
            self.position += goal_direction * speed * dt
        


    def draw(self, screen):
        pygame.draw.circle(screen, self.color, self.position, self.radius)


class GoalPoint(Drawable):
    def __init__(self, position, radius, color):
        self.position = position
        self.radius = radius
        self.color = color
    
    def draw(self, screen):
        pygame.draw.circle(screen, self.color, self.position, self.radius)


def generate_line(line_start: np.ndarray, line_end: np.ndarray, count_points, noise_offset):
    line_start = np.array(line_start)
    line_end = np.array(line_end)

    line_diff = line_end - line_start
    line_length = np.linalg.norm(line_diff)
    line_direction = line_diff / line_length

    segment_lenght = line_length / (count_points - 1)
    
    noise_direction = np.cross(np.array(list(line_direction) + [0]), np.array([0, 0, 1]))[:2]
    
    line = []
    for id in range(count_points):
        point = line_start + line_direction * segment_lenght * id
        
        noise = noise_direction * noise_offset * (np.random.random() * 2 - 1)
        point += noise

        line.append(point)
    
    return line
    

def draw_line(screen, line, color, thickness=2, point_radius=5):
    for idx in range(1, len(line.points)):
        pygame.draw.line(screen, color, line.points[idx - 1], line.points[idx], thickness)

    for point in line.points:
        pygame.draw.circle(screen, color, point, point_radius)


def generate_obstacles(count, min_raiuds, max_radius, screen_width, screen_height, palette):
    obstacles: List[Obstacle] = []
    for idx in range(count):
        radius = np.random.uniform(min_raiuds, max_radius)

        position_x = np.random.randint(0, screen_width)
        position_y = np.random.randint(0, screen_height)
        position = np.array([position_x, position_y])

        color = palette[np.random.randint(len(palette))]

        obstacle = Obstacle(position, radius, color)
        obstacles.append(obstacle)
    
    return obstacles
    


pygame.init()

screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Dynamic window approach")



top_line = Line(generate_line([0, screen_height / 8], [screen_width, screen_height / 8], 20, 10), (109, 0, 109))
middle_line = Line(generate_line([0, screen_height / 2], [screen_width, screen_height / 2], 20, 10), (85, 172, 238))
bottom_line = Line(generate_line([0, screen_height - screen_height / 8], [screen_width, screen_height - screen_height / 8], 20, 10), (0, 109, 109))

palette = [(49, 245, 173)]

robot = Robot(position=np.array([100, (screen_height - screen_height / 8) - (screen_height / 2 - screen_height / 8) / 2]),
              min_velocity=0,
              max_velocity=100,
              speed=10,
              omega_speed=2,
              distance_to_inter_move=30,
              radius=10, 
              color=(255, 174, 43))
goal = GoalPoint(position=np.array([screen_width - 100, 3 * screen_height / 4]), radius=10, color=(255, 43, 43))

obstacles: List[Obstacle] = [top_line, middle_line, bottom_line]
obstacles += generate_obstacles(25, 10, 15, screen_width, screen_height, palette)

drawable_objects: List[Drawable] = copy.copy(obstacles)
drawable_objects += [robot, goal]


dt = 0
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    t1 = time.time()
    # Отрисовка фона
    screen.fill((0, 0, 0))

    robot.move_to_goal(goal.position, obstacles, dt)
    robot.update_robot_state(goal.position, dt)

    for object in drawable_objects:
        object.update()

    for object in drawable_objects:
        object.draw(screen)

    # Обновление экрана
    pygame.display.flip()
    t2 = time.time()
    dt = t2 - t1

# Завершение работы Pygame
pygame.quit()
sys.exit()