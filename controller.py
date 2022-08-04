from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple

from robot import LinearRobot, LinearUnicycleKinematics

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

    # TODO remove list comprehensions for matrix powers
    def get_control(self, x: np.ndarray, t: float) -> np.ndarray:

        # Get the reference trajectory samples over the horizon
        x_r = self.traj.get_poses_lookahead(t, self.horizon, self.dt)
        N = x_r.shape[0]

        # Get the Jacobians of the model at the current state
        model : LinearRobot = LinearUnicycleKinematics.from_dt(x, self.dt)
        A = model.A
        B = model.B

        # Determine the matricies S and R that allow us to roll out the
        # predicted trajectory x over the horzion  
        # x = S x_t + R u_t:t+N 
        S = np.array( [np.linalg.matrix_power(A, n) for n in range(1, N+1)] ) # N x 3 x 3
        alpha = S @ x # N x 3
        # Compute the highest order row for final predicted state 
        R_nm1_b = np.tile(B, (N, 1, 1))
        R_nm1_a = np.array([ np.linalg.matrix_power(A, i) for i in range(N)])
        R_nm1 = R_nm1_a @ R_nm1_b # N x 2 x 2
        R_nm1 = R_nm1.reshape(N*2, -1).T # 2 x (2)(N)
        # Build the R matrix to get predictions at each step
        R = np.tile(R_nm1, (N, 1)) # (2)(N) x (2)(N)
        R = np.array([np.roll(row, x) for row, x in zip(R, np.arange(N)[::-1].repeat(2))])
        # Build block triangular mask and apply
        d = R.shape[0]//2
        mask = np.repeat(np.repeat(np.tril(np.ones((d, d))), 2, 1), 2, 0)
        R = R * mask

        # Formulate the QP in canonical form

        return np.array([0, 0])


class PoseVelMPC(Controller):
    def __init__(self):
        raise NotImplementedError()

    def get_control(self, x: np.ndarray, t: float) -> np.ndarray:
        raise NotImplementedError()
