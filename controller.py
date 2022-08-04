from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple

from robot import LinearRobot

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

    def __post_init__(self):
        end_pts: Tuple[np.ndarray, np.ndarray] = (self.poses[0], self.poses[-1])
        self.interp = interp1d(
            self.times, self.poses.T, bounds_error=False, fill_value=end_pts
        )

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
        return self.interp(sample_times)


class PoseMPC(Controller):
    def __init__(self, traj : Trajectory, Q : np.ndarray, horizon : float, dt : float):
        """
        Generates control inputs to make a robot track the given trajectory.
        Minimizes 1/2 e'Qe where e represents the vector of pose errors over
        the control horizon relative to the reference trajectory

        Args:
        traj - Trajectory you want to track
        Q - positive semi-definite matrix for quadratic cost in position
        """
        self.traj = traj
        self.Q = Q
        self.horizon = horizon 
        self.dt = dt


    def get_control(self, x: np.ndarray, t: float) -> np.ndarray:
        # Get the reference trajectory samples over the horizon
        x_r = traj.get_poses_lookahead(t, self.horizon, self.dt)
        N = x_r.shape[0]
        # Get the Jacobians of the model at the current state
        model : LinearRobot = LinearUnicycleKinematics.from_dt(x, self.dt)
        A = model.A
        B = model.B
        # Determine the matricies S and R that allow us to compute the
        # predicted trajectory x over the horzion  
        # x = S x_t + R u_t:t+N 
        S = np.linalg.matrix_power(A, n) 
        # Compute the highest order row for final predicted state 
        R_nm1_b = np.tile(B, (N, 1, 1))
        # TODO Is this doable in a vectorized way?
        R_nm1_a = np.array([ np.linalg.matrix_power(A, i) for i in range(N)])
        R_nm1 = A @ B
        R = np.tile(R_nm1, (N, 1))
        #R = [np.roll(R, 
        #R = 
        # mask R to be triangular
        return np.array([0, 0])


class PoseVelMPC(Controller):
    def __init__(self):
        raise NotImplementedError()

    def get_control(self, x: np.ndarray, t: float) -> np.ndarray:
        raise NotImplementedError()
