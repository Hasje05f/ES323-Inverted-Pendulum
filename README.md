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


### Bicopter
Two rotors are attached to the ends of pole that can rotate around its axis. It makes an angle `theta` with the horizontal.

<img src="Bicopter/bicopter.jpg" alt="drawing" width="400"/>

(See also [here](https://www.youtube.com/watch?v=VhRhwY8sl6A&ab_channel=XavierNeyt))

* state vars = [`theta`, `omega`]
* init_state = [0.0, 0.0] - in radians and radians/sec
* limits on state: -10.0° < `theta` < 10.0°
* input = the difference in voltage `V` that is applied to the motors that drive both rotors
* limits on input: -10.0 < `V` < 10.0 
* measurement: `theta`
* disturbance = a torque `T` [Nm] applied to the rotation axis.

### Ship
A ship is lying in a body of water. The mass at the end of a pole if mounted on top of the ship and can rotate (angle `phi`) around a vertical axis to steer the behavior of the ship around its axis (angle `theta`).
![Ship](Ship/ship.jpg)
* state vars = [`theta`, `omega`]
* limits on state: -1.0 < `theta` < 1.0
* init_state = [0.0, 0.0] - in radians and radians/sec
* input = angle `phi` [radians]
* limits on input: -pi/2 < `phi` < pi/2

<img src="Ship/ship2.jpg" alt="drawing" width="200"/>

* measurement: `theta`
* disturbance = an acceleration applied to the ship. Toggle this to simulate a push to the side of the ship.



### Ball On Beam
A small ball can roll over a beam. This beam can rotate around its axis and the rotation angle `alpha` can be set directly (i.e. no delay because of the servo-motor that rotates the plane).

<img src="Ball on Beam/bob.jpg" alt="drawing" width="200"/>
<img src="Ball on Beam/bob2.png" alt="drawing" width="400"/>

* state vars = [`x`, `v` ] - the position and speed of the ball
* limits on state: -1.0 < `x` < 1.0
* init_state = [1.0, 1.0]  - in [m] and [m/s]
* input = angle `alpha`
* limits on input: -10.0° < `alpha` < 10.0°
* measurement: position on beam `x` with a camera mounted above the setup - thus the projection of `x` in the horizontal plane.
* disturbance = an acceleration applied to the ball. Toggle this to simulate a push to the ball.

### Magnetic Levitation
A small iron ball is put inside a magnetic field. The strength of this magnetic field can be controlled by means of a current `I` through an inductance.

<img src="Magnetic Levitation/maglev.jpg" alt="drawing" height="400"/>

* state vars = [`z`, `v`] where `z` is the vertical distance ([cm]), measured from the top of the box in which the ball is confined. `v` is the vertical velocity.
* limits on state: 0.0 < `z` < 10.0
* init_state = [5.0, 0.0]
* input = current `I` [A]
* limits on input: 0.0 < `I` < 30.0
* disturbance: the `set_disturbance` function here limits the minimum value of `z`. This is needed to do the identification and parameter estimation of the system.

## Inverted Pendulum  
The classic inverted pendulum, where you must move a cart to keep the pendulum in a vertical position.

<img src="Inverted Pendulum/invertedpendulum.jpg" alt="drawing" height="400"/>

* state vars = [`x`, `v`, `theta`, `omega`] with `x` [m] and `v` [m/s] the position and velocity of the cart, and `theta` and `omega` the angle [rad] and angular velocity [rad/s] of the pendulum with respect to the vertical.
* limits on state: -1 < `x` < 1 and -pi/2 < `theta` < pi/2
* init_state = [0.0, 0.0, 0.0, 0.0]
* input = a force `F` [N] applied to the cart
* limits on input: -10 < `F` < 10
* !! Pendulum returns two measurements: postion of the cart `x` and angle `theta`, so `y` is a list