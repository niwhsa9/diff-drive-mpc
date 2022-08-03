from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass

from sim import Sim

import numpy as np
import pygame

@dataclass
class Robot(ABC):
    """
    Generic robot dynamics model x' = f(x, u)
    """
    x : np.ndarray

    @abstractmethod
    def draw(self, sim: Sim, screen : pygame.Surface):
        """
        Draws the robot on the screen
        """

    @abstractmethod
    def update_state(self, u : np.ndarray, dt : float) -> np.ndarray:
        """
        Updates the internal state and returns the next state via
        whatever numerical ODE method is chosen

        Args:
        u - control input
        dt - elapsed time since previous polling
        """

@dataclass
class LinearRobot(Robot):
    """
    Discrete time LTI dynamics model x' = Ax + Bu
    """
    A : np.ndarray
    B : np.ndarray

    def __init__(self, A, B):
        self.A = A
        self.B = B

    def update_state(self, u : np.ndarray, _) -> np.ndarray:
        """
        Args:
        u - state space control input
        """
        return self.A @ self.x + self.B @ u

@dataclass
class UnicycleKinematics(Robot):
    """
    The continious time unicycle kinematics are as follows.

    x' = v cos (theta)
    y' = v sin (theta)
    theta' = omega

    With control inputs: v, omega. 
    ref: http://www.ece.ufrgs.br/~fetter/sbai05_10022.pdf
    """

    def update_state(self, u : np.ndarray, dt : float) -> np.ndarray:
        '''
        Provides a first-order discrete time Euler integration update
        '''
        self.x[0] += u[0] * np.cos(self.x[2]) * dt
        self.x[1] += u[0] * np.sin(self.x[2]) * dt
        self.x[2] += u[1] * dt
        return self.x


@dataclass
class LinearUnicycleKinematics(LinearRobot):
    """
    This is a linearization of the unicycle kinematics
    """
    @classmethod
    def from_dt(cls, dt : float) -> UnicycleKinematicsRobot:
        pass

