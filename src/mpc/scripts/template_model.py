import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc

def template_model():

    model_type = 'continuous'  # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    car_x = model.set_variable(var_type='_x', var_name='car_x', shape=(1, 1))
    car_y = model.set_variable(var_type='_x', var_name='car_y', shape=(1, 1))
    car_theta = model.set_variable(var_type='_x', var_name='car_theta', shape=(1, 1))

    car_v = model.set_variable(var_type='_u', var_name='car_v')
    car_delta = model.set_variable(var_type='_u', var_name='car_delta')

    LR = 0.17145
    LF = 0.15875
    WHEELBASE_LENGTH = 0.3302   

    # Next state equations - these mathematical formulas are the core of the model
    slip_factor = casadi.arctan(LR * casadi.tan(car_delta) / WHEELBASE_LENGTH)
    model.set_rhs("car_x", car_v * casadi.cos(car_theta + slip_factor))
    model.set_rhs("car_y", car_v * casadi.sin(car_theta + slip_factor))
    model.set_rhs("car_theta", car_v * casadi.tan(car_delta)* casadi.cos(slip_factor) / WHEELBASE_LENGTH)

    # Create time-varying-parameters, these will be populated with (potentially) different data at each call
    model.set_variable(
            var_type="_tvp",
            var_name="target_x",
            shape=(1, 1)
    )
    model.set_variable(
            var_type="_tvp",
            var_name="target_y",
            shape=(1, 1)
    )

    # Build the model
    model.setup()

    # Obtain an instance of the do-mpc model class
    # and select time discretization:

    # model_type = 'continuous' # either 'discrete' or 'continuous'
    # model = do_mpc.model.Model(model_type)

    # # States struct (optimization variables):
    # X_s = model.set_variable('_x',  'X_s') # Vehicle X position 
    # Y_s = model.set_variable('_x',  'Y_s') # Vehicle Y position
    # V_s = model.set_variable('_x',  'V_s') # Vehicle velocity
    # Theta_s = model.set_variable('_x',  'Theta_s') # Vehicle yaw angle map frame

    # # The control inputs are steering angle and throttle
    # u_throttle = model.set_variable('_u',  'u_throttle')
    # u_steering = model.set_variable('_u',  'u_steering') # in radians

    # # System Identification Parameters 
    # ca = 1.9569     # acceleration constant
    # cm = 0.0342     # motor constant
    # ch = -37.1967   # alleged hysteresis constant 
    # lf = 0.225      # distance from car center of mass to front
    # lr = 0.225      # distance from car center of mass to rear

    # # Directly from the Differential equations. See paper
    # model.set_rhs('X_s', V_s * cos(Theta_s))
    # model.set_rhs('Y_s', V_s* sin(Theta_s))
    # model.set_rhs('V_s', (-ca*V_s)+(ca*cm*(u_throttle-ch)))
    # model.set_rhs('Theta_s', (V_s/(lf+lr))*tan(u_steering))

    # # Build the model
    # model.setup()

    return model