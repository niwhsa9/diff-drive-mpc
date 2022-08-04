from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple

import numpy as np
from scipy.interpolate import interp1d


class Controller(ABC):
    @abstractmethod
    def get_control(self, x: np.ndarray, t: float) -> np.ndarray:
        pass


class DummyController(Controller):
    def get_control(self, x, t) -> np.ndarray:
        return np.array([0, 0])


@dataclass
class Trajectory:
    # N x 3 array of poses
    poses: np.ndarray
    # N timestamps to match the poses
    times: np.ndarray
    # N x 3 array of velocities
    velocities: np.ndarray

    def get_poses_lookahead(
        self, start_time: float, duration: float, dt: float
    ) -> np.ndarray:
        """
        Retrieves the poses of the trajectory from the start time to
        start time + duration at the given time discretization by 
        linearly interpolating between the specified points of the 
        trajectory.

        """
        sample_times = np.arange(start_time, start_time + duration, dt)
        end_pts: Tuple[np.ndarray, np.ndarray] = (self.poses[0], self.poses[-1])
        # TODO can be class member, maybe use __post_init__??
        interp = interp1d(
            self.times, self.poses.T, bounds_error=False, fill_value=end_pts
        )
        return interp(sample_times)


class PoseMPC(Controller):
    def __init__(self):
        pass

    def get_control(self, x: np.ndarray, t: float) -> np.ndarray:

        return np.array([0, 0])


class PoseVelMPC(Controller):
    def __init__(self):
        pass

    def get_control(self, x: np.ndarray, t: float) -> np.ndarray:

        return np.array([0, 0])
