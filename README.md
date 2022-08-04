This repo contains two things 
- A general framework for robot dynamical system simulation, visualization, and controller testing
- A fast linear MPC controller for trajectory tracking with differential drives that is solved via a quadratic program

References:
- For the differential drive MPC, the unicycle jacobian and QP formulation are primarily based on the method in this paper: http://www.ece.ufrgs.br/~fetter/sbai05_10022.pdf
- The formulation of my pose-only MPC is included below
- $$\sum$$
