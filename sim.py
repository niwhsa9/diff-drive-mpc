from dataclasses import dataclass
from typing import Tuple, Sequence

from robot import Robot, UnicycleKinematics
from controller import Controller, DummyController

import numpy as np
import pygame

@dataclass
class Sim:
    '''
    Grapical simulator implementation for testing controllers on 
    robot models with no state estimation noise.
    '''
    sim_freq : float
    control_freq : float
    robot : Robot
    controller : Controller
    # World dims in meters
    world_dims : Tuple[int, int] = (30, 30)
    # Screen dims in pixels
    screen_dims : Tuple[int, int] = (500, 500)
    # Robot size in meters, RH coordinate with +X aligned to 'front'
    robot_size : Tuple[int, int] = (2, 1)

    def world_to_screen_pos(self, world_pos : np.ndarray) -> np.ndarray:
        pass

    def world_to_screen_dims(self, world_dims : np.ndarray) -> np.ndarray:
        return world_dims * np.array(self.screen_dims)/np.array(self.world_dims)

    def draw(self, screen: pygame.surface.Surface) -> None:
        screen.fill((0, 0, 0))

        # Display the robot
        #robot_pos = self.world_to_screen_pos(robot.get_drawable())
        screen.blit(self.robot_sprite, self.robot_sprite.get_rect())

            
        pygame.display.flip()

    def run(self):
        pygame.init()
        screen : pygame.surface.Surface = pygame.display.set_mode(self.screen_dims)

        self.robot_sprite = pygame.transform.scale(pygame.image.load("arrow.jpg"), self.world_to_screen_dims(self.robot_size))
        # self.robot_sprite.convert()
        # self.robot_sprite.set_color_key((0, 0, 0))

        running : bool = True

        sim_ticks_per_control : int = int(round(self.sim_freq / self.control_freq))
        sim_dt : float = 1.0 / self.sim_freq
        time = 0

        while running:
            # Check for closing event
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
           
            # Get control input
            u = controller.get_control(robot.x)

            # Update the simulated state
            for _ in range(sim_ticks_per_control):
                robot.update_state(u, sim_dt)
                time += sim_dt

            # Graphics
            self.draw(screen)
            
        pygame.quit()

if __name__ == "__main__":
    robot = UnicycleKinematics(np.array([0, 0, 0]))
    controller = DummyController()
    sim = Sim(240, 60, robot, controller)
    sim.run()
