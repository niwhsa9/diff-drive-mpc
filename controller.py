from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


class Controller(ABC):
    @abstractmethod
    def get_control(self, x: np.ndarray, t : float) -> np.ndarray:
        pass


class DummyController(Controller):
    def get_control(self, x, t) -> np.ndarray:
        return np.array([0, 0])

@dataclass
class Trajectory:
    # N x 3 array of poses
    poses : np.ndarray
    # N timestamps to match the poses
    times : np.ndarray
    # N x 3 array of velocities
    velocities : np.ndarray

    def get_poses_lookahead(start_time : float, duration : float, dt : float) -> np.ndarray:
        """
        Retrieves the poses of the trajectory from the start time to
        start time + duration at the given time discretization by 
        linearly interpolating between the specified points of the 
        trajectory.

        """

    
    

class MPC(Controller):
    def __init__(self):
        pass

    def get_control(self, _) -> np.ndarray:

        return np.array([0, 0])
