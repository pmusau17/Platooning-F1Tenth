import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc


def template_mpc(model, tarx, tary, a_b, b_b, a_u, b_u, flag_b, flag_u):

    mpc = do_mpc.controller.MPC(model)

    setup_mpc = {
        'n_horizon': 5,
        'n_robust': 0,
        't_step': 0.1,
        'state_discretization': 'collocation',
        'store_full_solution': False,
    }

    mpc.set_param(**setup_mpc)

    _x = model.x


    # mterm = np.sqrt(((tarx- _x['car_x']) ** 2) + ((tary - _x['car_y']) ** 2))
    # lterm = np.sqrt(((tarx - _x['car_x']) ** 2) + ((tary - _x['car_y']) ** 2))

    # mterm = ((tarx- _x['car_x']) ** 2) + ((tary - _x['car_y']) ** 2)
    # lterm = ((tarx - _x['car_x']) ** 2) + ((tary - _x['car_y']) **2)

    mterm = (fabs(tarx- _x['car_x'])) + (fabs(tary - _x['car_y']))
    lterm = (fabs(tarx - _x['car_x'])) + (fabs(tary - _x['car_y']))

    mpc.set_objective(mterm=mterm, lterm=lterm)
    #mpc.set_rterm(car_v= , car_delta=0.3)

    # These parameters are very sensitive 
    mpc.set_rterm(car_v=0.9 , car_delta=0.2)

    mpc.bounds['lower', '_u', 'car_v'] = 0.3
    mpc.bounds['lower', '_u', 'car_delta'] = -0.6189


    mpc.bounds['upper', '_u', 'car_v'] = 0.5
    mpc.bounds['upper', '_u', 'car_delta'] = 0.6189


    #mpc.scaling['_u', 'car_v'] = 2
    #mpc.scaling['_u', 'car_delta'] = 1
    
    mpc.set_nl_cons('constraint_bottom',  -(a_b * _x['car_x'] + b_b - _x['car_y']), 0, penalty_term_cons=1000)
    mpc.set_nl_cons('constraint_upper',   (a_u * _x['car_x'] + b_u - _x['car_y']) , 0, penalty_term_cons=1000)

    mpc.setup()


    return mpc
