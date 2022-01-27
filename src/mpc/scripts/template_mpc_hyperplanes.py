import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc


def template_mpc(model,horizon): 

    mpc = do_mpc.controller.MPC(model)
    setup_mpc = {
        'n_horizon': horizon,
        'n_robust': 0,                         # Robust horizon for robust scenario-tree MPC,
        'open_loop': 0,
        't_step': 0.01,
        'state_discretization': 'collocation', # no other option at the moment
        'collocation_type': 'radau',           # no other option at the moment
        'collocation_deg': 2,
        'collocation_ni': 2,
        'store_full_solution': False, # re
        #'nlpsol_opts': {'ipopt.linear_solver': 'MA27'} # highly recommended, Use MA27 linear solver in ipopt for faster calculations
    }

    mpc.set_param(**setup_mpc)

    _x = model.x
    _tvp = model.tvp

    # change the objective function to a time varying parameter
    # lterm = ((_tvp['target_x']- _x['car_x']) ** 2) + ((_tvp['target_y'] - _x['car_y']) ** 2)

    lterm = DM.zeros()
    mterm = ((_tvp['target_x']- _x['car_x']) ** 2) + ((_tvp['target_y'] - _x['car_y']) ** 2)

    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(car_v=5.0 , car_delta=0.1)

    mpc.bounds['lower', '_u', 'car_v'] = 0.0
    mpc.bounds['lower', '_u', 'car_delta'] = -0.6189
    mpc.bounds['upper', '_u', 'car_v'] = 0.5
    mpc.bounds['upper', '_u', 'car_delta'] = 0.6189

    # left plane constraint, this basically bounds it to the right or to the left
    mpc.set_nl_cons('constraint_left_plane',  _tvp['c1'] * ((_tvp['a0'] * _x['car_x'] + _tvp['b0']) - _x['car_y']), 0)

    # left plane constraint, this basically bounds it to the right or to the left
    mpc.set_nl_cons('constraint_right_plane', _tvp['c2'] * ((_tvp['a1'] * _x['car_x'] + _tvp['b1']) - _x['car_y']), 0)
    
    return mpc
