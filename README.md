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
The classic inverted pendulum, where you must move a cart to keep the pendulum in a vertical position.

<img src="Inverted Pendulum/invertedpendulum.jpg" alt="drawing" height="400"/>

* state vars = [`x`, `v`, `theta`, `omega`] with `x` [m] and `v` [m/s] the position and velocity of the cart, and `theta` and `omega` the angle [rad] and angular velocity [rad/s] of the pendulum with respect to the vertical.
* limits on state: -1 < `x` < 1 and -pi/2 < `theta` < pi/2
* init_state = [0.0, 0.0, 0.0, 0.0]
* input = a force `F` [N] applied to the cart
* limits on input: -10 < `F` < 10
* !! Pendulum returns two measurements: postion of the cart `x` and angle `theta`, so `y` is a list
