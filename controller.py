from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple

from robot import LinearRobot, LinearUnicycleKinematics

import numpy as np
from scipy.interpolate import interp1d
from scipy.linalg import block_diag
from qpsolvers import solve_qp


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
        return self.interp(sample_times).T


class PoseMPC(Controller):
    def __init__(self, traj : Trajectory, Q : np.ndarray, max_vel : np.ndarray, horizon : float, dt : float):
        """
        Generates control inputs to make a robot track the given trajectory.
        Minimizes 1/2 e'Qe where e represents the vector of pose errors over
        the control horizon relative to the reference trajectory

        Args:
        traj - Trajectory you want to track
        Q - positive semi-definite matrix for quadratic cost in position
        """
        N = int(np.floor(horizon/dt))
        self.traj = traj
        self.Q = block_diag(*[Q for _ in range(N)])
        self.horizon = horizon 
        self.dt = dt
        self.max_vel = max_vel
        self.prev_u = np.zeros(2)

        self.G = np.vstack((np.eye(N*2),np.eye(N*2)))
        self.h = np.tile(self.max_vel, N*2)

    # TODO vectorize list comprehensions for matrix powers
    def get_control(self, x: np.ndarray, t: float) -> np.ndarray:

        # Get the reference trajectory samples over the horizon
        x_r = self.traj.get_poses_lookahead(t, self.horizon, self.dt) # N x 3
        N = x_r.shape[0]

        # Get the Jacobians of the model at the current state
        model : LinearRobot = LinearUnicycleKinematics.from_dt(x, self.prev_u, self.dt)
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
        R_nm1 = R_nm1_a @ R_nm1_b # N x 3 x 2
        R_nm1 = R_nm1.reshape(N*2, -1).T # 3 x (2)(N)
        # Build the R matrix to get predictions at each step
        R = np.tile(R_nm1, (N, 1)) # (3)(N) x (2)(N)
        R = np.array([np.roll(row, x) for row, x in zip(R, -np.arange(N)[::-1].repeat(3))])
        # Build block triangular mask and apply
        mask = np.repeat(np.repeat(np.tril(np.ones((N, N))), 2, 1), 3, 0)
        R = R * mask

        # Formulate the QP in canonical form 
        # 1/2 x'Px + q'x
        # G x <= h
        P = R.T @ self.Q @ R
        q = (alpha - x_r).reshape(-1) @ (self.Q @ R)
        u_optimal = solve_qp(P, q, self.G, self.h, None, None, solver="cvxopt")
        if u_optimal is None:
            u_optimal = np.array([0, 0])
        self.prev_u = u_optimal
        return u_optimal


class PoseVelMPC(Controller):
    def __init__(self):
        raise NotImplementedError()

    def get_control(self, x: np.ndarray, t: float) -> np.ndarray:
        raise NotImplementedError()
