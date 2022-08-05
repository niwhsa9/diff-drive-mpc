## MPC Trajectory Tracker
**Contents** 

This repository contains two things
- A general framework for controller testing including robot dynamical system simulation and visualization. There is no state estimation noise, though the simulated dynamics are the true non-linear dynamics with additive gaussian noise
- A fast linear MPC for trajectory tracking on differential drives that is solved via a quadratic program

References:
- The unicycle jacobian, exponential cost increase, and QP formulation are based on the method in this paper: http://www.ece.ufrgs.br/~fetter/sbai05_10022.pdf
- http://underactuated.mit.edu/index.html was generally helpful to me
- https://github.com/michiganrobotics/rob101/blob/main/Fall%202020/Projects/Project-03/ derrives a simplification for repeated application of the state space propogation in discrete time
- The formulation of the position-only trajectory tracking MPC is included below. It is inspired by what is done in the linked work but not explicitly derived.

**Derivation**

I seek to find
$$
u^* = \argmin_u \frac{1}{2}\tilde{x}^T Q \tilde{x}
$$
where $\tilde{x} =x-x_{r}$. I use $x = \begin{bmatrix} x(t) \\ x(t+\Delta t) \\ ... \\ x(t+T) \end{bmatrix}$ to refer to the vector of state vectors over the entire horizon, t $\rightarrow$ t + T with discretization $\Delta t$. Likewise, $x_r$ and $u$ follow the same notation. Both $x$ and $x_r$ are of length $N$.

I compute the linearized system dynamics $A$ and $B$ matricies from the state estimate at the start of the horizon.

Propogating the state space update from the initial state $x(t)$ lets me obtain
$$
x(t+k\Delta t) = \prod_{i=1}^{k} (A) \ x(t) + \begin{bmatrix} A^{k-1}B & A^{k-2} B & ... & AB & B\end{bmatrix} u(t + (k-1)\Delta t)
$$

By computing all states for the whole horizon, I can denote the predicted trajectory as:
$$
x = \alpha + R u
$$
I now substitute this into my original quadratic cost to get a QP over $u$ in canonical form. Letting $f=\alpha - x_r$
$$
\begin{align*}
u^* &= \argmin_u \frac{1}{2} (\alpha - x_r+Ru)Q(\alpha - x_r + Ru) \\
 &= \argmin_u \frac{1}{2} (f + Ru)Q(f + Ru) \\
 &= \argmin_u (\frac{1}{2}f^T Q + \frac{1}{2}u^TRQ)(f+Ru)\\
 &= \argmin_u \frac{1}{2}f^TQf+\frac{1}{2}f^TQRu+\frac{1}{2}u^TR^TQf+\frac{1}{2}u^TR^TQRu\\
 &= \argmin_u f^TQRu+\frac{1}{2}u^TR^TQRu
\end{align*}
$$
 From here the mapping to the canonical form for CVXOPT is given as
$$
P := R^T Q R \\
q := (\alpha -x_r)QR
$$