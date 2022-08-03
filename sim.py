from dataclasses import dataclass
from typing import Tuple

from robot import Robot, UnicycleKinematics
from controller import Controller

import numpy as np
import pygame

@dataclass
class Sim:
    sim_freq : float
    control_freq : float
    robot : Robot
    controller : Controller
    # World dims in meters
    world_dims : Tuple[int, int]
    # Screen dims in pixels
    screen_dims : Tuple[int, int] = (500, 500)

    def world_to_screen_pos(self, world_pos : np.ndarray) -> np.ndarray:
        pass

    def world_to_screen_dims(self, world_pos : np.ndarray) -> np.ndarray:
        pass

    def draw(self, screen: pygame.Surface):
        screen.fill((255, 255, 255))
        pygame.display.flip()

    def run(self):
        pygame.init()
        screen : pygame.Surface = pygame.display.set_mode(self.screen_dims)

        running : bool = True

        sim_ticks_per_control : int = np.round(self.sim_freq / self.control_freq)
        sim_dt : float = 1.0 / self.sim_freq
        time = 0

        while running:
            # Check for closing event
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
           
            # Get control input
            u = controller.

            # Update the simulated state
            for _ in range(sim_ticks_per_control):
            
                robot.update_state(sim_dt, u)
                time += sim_dt

            # Graphics
            self.draw(screen)
            
        pygame.quit()

if __name__ == "__main__":
    robot = UnicycleKinematics(np.array([0, 0]))
    controller = Controller()
    sim = Sim(240, 60, robot, controller, (30, 30))
    sim.run()
