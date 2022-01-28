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
    car_time = model.set_variable(var_type='_x', var_name='time', shape=(1, 1))

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
    model.set_rhs("time",car_v/car_v)

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

    model.set_variable(
            var_type="_tvp",
            var_name="target_theta",
            shape=(1, 1)
    )

    model.set_variable(
            var_type="_tvp",
            var_name="a0",
            shape=(1, 1)
    )
    model.set_variable(
            var_type="_tvp",
            var_name="b0",
            shape=(1, 1)
    )
    # Create time-varying-parameters, these will be populated with (potentially) different data at each call
    model.set_variable(
            var_type="_tvp",
            var_name="a1",
            shape=(1, 1)
    )
    model.set_variable(
            var_type="_tvp",
            var_name="b1",
            shape=(1, 1)
    )
    
    # Create time-varying-parameters, these will be populated with (potentially) different data at each call
    model.set_variable(
            var_type="_tvp",
            var_name="c1",
            shape=(1, 1)
    )
    model.set_variable(
            var_type="_tvp",
            var_name="c2",
            shape=(1, 1)
    )


#     My Attempt at using time varying parameters to change constraints online
#     Create time-varying-parameters, these will be populated with (potentially) different data at each call
#     These will be leveraged in the non_linear constraints. They will actually be linear but this allows 
#     you to change the bounds online with time varying parameters.
    model.set_variable(
            var_type="_tvp",
            var_name="x_min",
            shape=(1, 1)
    )
    model.set_variable(
            var_type="_tvp",
            var_name="x_max",
            shape=(1, 1)
    )
    # Create time-varying-parameters, these will be populated with (potentially) different data at each call
    model.set_variable(
            var_type="_tvp",
            var_name="y_min",
            shape=(1, 1)
    )
    model.set_variable(
            var_type="_tvp",
            var_name="y_max",
            shape=(1, 1)
    )


    # Build the model
    model.setup()

    return model