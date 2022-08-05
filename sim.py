from __future__ import annotations
from dataclasses import dataclass
from typing import List, Callable, Union, Any

from robot import Robot, UnicycleKinematics
from controller import Controller, DummyController, Trajectory, PoseMPC

import numpy as np
import pygame


@dataclass
class Sim:
    """
    Grapical simulator implementation for testing controllers on 
    robot models with no state estimation noise.
    """

    sim_freq: float
    control_freq: float
    robot: Robot
    controller: Controller
    # World dims in meters
    world_dims: np.ndarray = np.array([30, 30])
    # Screen dims in pixels
    screen_dims: np.ndarray = np.array([500, 500])
    # Robot size in meters, RH coordinate with +X aligned to 'front'
    robot_size: np.ndarray = np.array([2, 1])
    drawables: Union[List[Callable[[Sim], Any]], None] = None

    def world_to_screen_pos(self, world_pos: np.ndarray) -> np.ndarray:
        """
        Converts an Nx2 array of world positions to screen positions
        """
        pos = np.copy(world_pos)
        # This inversion is necessary due to standard y down image coordinates
        if len(pos.shape) == 1:
            pos[1] *= -1
        else:
            pos[:, 1] *= -1
        return pos * self.screen_dims / self.world_dims + self.screen_dims / 2

    def world_to_screen_dims(self, world_dims: np.ndarray) -> np.ndarray:
        """
        Converts an Nx2 array of world dimensions to screen dimensions
        """
        return world_dims * np.array(self.screen_dims) / np.array(self.world_dims)

    def draw(self) -> None:
        # Blank background
        self.screen.fill((0, 0, 0))

        # Display the robot
        robot_pos, robot_theta = robot.get_drawable()
        robot_sprite = pygame.transform.rotate(
            self.robot_sprite, np.rad2deg(robot_theta)
        )
        hitbox = robot_sprite.get_rect(
            center=tuple(self.world_to_screen_pos(robot_pos))
        )
        self.screen.blit(robot_sprite, hitbox)

        # Draw user defined things
        if self.drawables:
            [f(self) for f in self.drawables]

        # Flip buffers to draw graphics
        pygame.display.flip()

    def run(self) -> None:
        pygame.init()
        self.screen: pygame.surface.Surface = pygame.display.set_mode(
            tuple(self.screen_dims)
        )

        self.robot_sprite = pygame.transform.scale(
            pygame.image.load("resources/arrow.jpg"),
            tuple(self.world_to_screen_dims(self.robot_size)),
        )
        # Necessary for alpha
        self.robot_sprite.convert()
        self.robot_sprite.set_colorkey((0, 0, 0))

        running: bool = True

        sim_ticks_per_control: int = int(round(self.sim_freq / self.control_freq))
        sim_dt: float = 1.0 / self.sim_freq
        time: float = 0

        while running:
            # Check for closing event
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Get control input
            # There is no state estimation noise (for now)
            # the controller gets the ground truth state
            # u = np.array([0.1, 0.05])
            u = self.controller.get_control(robot.x, time)

            # Update the simulated state
            for _ in range(sim_ticks_per_control):
                robot.update_state(u, sim_dt)
                time += sim_dt

            # Graphics
            self.draw()

        pygame.quit()


if __name__ == "__main__":
    robot: Robot = UnicycleKinematics(np.array([0.0, 0, 0]))
    traj: Trajectory = Trajectory(
        np.array([
                    [0, 0, 0], [10, 0, 0], 
                    [10, 1, np.pi / 2], [10, 10, np.pi / 2],
                    [9, 10, np.pi], [-10, 10, np.pi]
                
                ]),
        np.array([0, 10, 11, 20, 21, 40]),
        np.zeros((3, 3)),
    )
    controller: Controller = PoseMPC(
        traj,
        np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 50.0]]),
        np.array([1.0, 0.2]),
        0.2,
        0.05,
    )
    draw_traj = lambda sim: pygame.draw.lines(
        sim.screen,
        (255, 0, 0),
        False,
        sim.world_to_screen_pos(traj.poses[:, 0:2]).tolist(),
    )
    sim = Sim(240, 60, robot, controller, drawables=[draw_traj])
    sim.run()
