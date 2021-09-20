import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc


def template_mpc(model, tarx, tary):

    mpc = do_mpc.controller.MPC(model)

    setup_mpc = {
        'n_horizon': 5,
        'n_robust': 0,
        't_step': 0.1,
        'state_discretization': 'discrete',
        'store_full_solution': True,
    }

    mpc.set_param(**setup_mpc)

    _x = model.x


    mterm = np.sqrt(((tarx- _x['car_x']) ** 2) + ((tary - _x['car_y']) ** 2))
    lterm = np.sqrt(((tarx - _x['car_x']) ** 2) + ((tary - _x['car_y']) ** 2))/5

    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(car_v= .9, car_delta=1)

    mpc.bounds['lower', '_u', 'car_v'] = .5
    mpc.bounds['lower', '_u', 'car_delta'] = -0.4189

    mpc.bounds['upper', '_u', 'car_v'] = 4
    mpc.bounds['upper', '_u', 'car_delta'] = 0.4189

    #mpc.scaling['_u', 'car_v'] = 2
    #mpc.scaling['_u', 'car_delta'] = 1

    mpc.setup()


    return mpc
