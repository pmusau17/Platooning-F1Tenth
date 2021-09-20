import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc

def template_model():

    model_type = 'discrete'  # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    car_x = model.set_variable(var_type='_x', var_name='car_x', shape=(1, 1))
    car_y = model.set_variable(var_type='_x', var_name='car_y', shape=(1, 1))
    car_theta = model.set_variable(var_type='_x', var_name='car_theta', shape=(1, 1))

    car_v = model.set_variable(var_type='_u', var_name='car_v')
    car_delta = model.set_variable(var_type='_u', var_name='car_delta')

    L = 0.3
    ts = 0.1

    model.set_rhs('car_x', car_x + (car_v * casadi.cos(car_theta + car_delta)) * ts)
    model.set_rhs('car_y', car_y + (car_v * casadi.sin(car_theta + car_delta)) * ts)
    model.set_rhs('car_theta', car_theta + ((car_v * casadi.sin(car_delta))/L) * ts)

    # Build the model
    model.setup()

    return model