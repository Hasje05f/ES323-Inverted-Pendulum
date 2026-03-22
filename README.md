# ES323 - Control Systems project.


## Requirements:
* MATLAB
* python 3 (3.6 or higher)
* numpy
* matplotlib (for visualization)

## General instructions:
1. Download the files, either with `git clone` (preferred) or by downloading all the files in zip format (choose `Clone or download` in the top right corner).
1. Unzip the files (if needed). Open a command window and `cd` into the newly created directory.
1. Use the MATLAB file "parameters" to create a JSON file used by python
1. Start the system simulator: `python system.py [bob|ship|maglev|bicopter|pendulum]`.  
1. Read the .csv files in MATLAB and plot


## Inverted Pendulum  
# Inverted pendulum

**Description:**  
An inverted pendulum consists of a cart on which a rod is fixed. The rod can rotate around an axis perpendicular to the direction of motion of the cart. The goal is to maintain the rod vertically, using the motion of the cart to compensate for external forces and maintain equilibrium. In other words, in steady-state, the rod is vertical and makes an angle θ = 0 with the vertical. Obviously that situation is not stable and the goal of the controller is to maintain θ = 0 by moving the cart. There is a sensor measuring the angle θ and another measuring the position of the cart x. Design a controller for this system such that the angle of the pendulum can be controlled and that the closed loop behaves appropriately in presence of disturbances.

* state vars = [`x`, `v`, `theta`, `omega`] with `x` [m] and `v` [m/s] the position and velocity of the cart, and `theta` and `omega` the angle [rad] and angular velocity [rad/s] of the pendulum with respect to the vertical.
* limits on state: -1 < `x` < 1 and -pi/2 < `theta` < pi/2
* init_state = [0.0, 0.0, 0.0, 0.0]
* input = a force `F` [N] applied to the cart
* limits on input: -10 < `F` < 10
* !! Pendulum returns two measurements: postion of the cart `x` and angle `theta`, so `y` is a list
