from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple

import numpy as np

@dataclass
class Robot(ABC):
    """
    Generic robot dynamics model x' = f(x, u)
    """
    x : np.ndarray

    @abstractmethod
    def update_state(self, u : np.ndarray, dt : float) -> np.ndarray:
        """
        Updates the internal state and returns the next state via
        whatever numerical ODE method is chosen

        Args:
        u - control input
        dt - elapsed time since previous polling
        """
    @abstractmethod
    def get_drawable(self) -> Tuple[float, float, float]:
        pass

@dataclass
class LinearRobot(Robot):
    """
    Discrete time LTI dynamics model x' = Ax + Bu
    """
    A : np.ndarray
    B : np.ndarray

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
    """
    def get_drawable(self) -> Tuple[float, float, float]:
        return tuple(self.x)

    def update_state(self, u : np.ndarray, dt : float) -> np.ndarray:
        """
        Provides a first-order discrete time Euler integration update
        """
        self.x[0] += u[0] * np.cos(self.x[2]) * dt
        self.x[1] += u[0] * np.sin(self.x[2]) * dt
        self.x[2] += u[1] * dt
        return self.x


@dataclass
class LinearUnicycleKinematics(LinearRobot):
    """
    This is a linearization of the unicycle kinematics
    """
    def get_drawable(self) -> Tuple[float, float, float]:
        return tuple(self.x)

    @classmethod
    def from_dt(cls, x: np.ndarray, dt : float) -> LinearUnicycleKinematics:
        return LinearUnicycleKinematics(x, np.eye(3) * dt, np.eye(3) * dt)
