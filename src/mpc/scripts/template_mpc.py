import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc


def template_mpc(model, horizon, mpc_x_min, mpc_y_min, mpc_x_max, mpc_y_max):

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

    #lterm = ((_tvp['target_x']- _x['car_x']) ** 2) + ((_tvp['target_y'] - _x['car_y']) ** 2)
    lterm = casadi.DM.zeros()
    mterm = ((_tvp['target_x']- _x['car_x']) ** 2) + ((_tvp['target_y'] - _x['car_y']) ** 2) + 0.1 * ((_tvp['target_theta'] - _x['car_theta'])**2)

    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(car_v=30.0 , car_delta=0.0)

    mpc.bounds['lower', '_u', 'car_v'] = 0.0
    mpc.bounds['lower', '_u', 'car_delta'] = -0.6189
    mpc.bounds['upper', '_u', 'car_v'] = 1.5
    mpc.bounds['upper', '_u', 'car_delta'] = 0.6189


    #right of left plane 
    mpc.set_nl_cons('constraint_bottom',  (_tvp['a0'] * _x['car_x'] + _tvp['b0']) - _x['car_y'], 0)

    #left of right plane 
    mpc.set_nl_cons('constraint_upper',   _x['car_y']  - (_tvp['a1'] * _x['car_x'] + _tvp['b1']), 0) 


    # # set the default values for the tvp parameters
    # tvp_struct = mpc.get_tvp_template()
    # for k in range(horizon + 1):
    #     tvp_struct["_tvp", k, "x_min"] = float(mpc_x_min)
    #     tvp_struct["_tvp", k, "x_max"] = float(mpc_x_max)
    #     tvp_struct["_tvp", k, "y_min"] = float(mpc_y_min)
    #     tvp_struct["_tvp", k, "y_max"] = float(mpc_y_max)

    #     print(tvp_struct["_tvp", k, "x_min"],tvp_struct["_tvp", k, "x_max"],
    #             tvp_struct["_tvp", k, "y_min"],tvp_struct["_tvp", k, "y_max"])

    # Set the boundary constraints, which just end up being linear constraints
    # x<=x_max becomes x - xmax <= 0 
    # x>=x_min become  x_min - x <=0 
    # and liewise for y 

    # mpc.set_nl_cons('x_lb', _tvp['x_min'] - _x['car_x'], ub=0) #, soft_constraint=True, penalty_term_cons=1e2)
    # mpc.set_nl_cons('x_ub', _x['car_x'] - _tvp['x_max'], ub=0) #, soft_constraint=True, penalty_term_cons=1e2)
    # mpc.set_nl_cons('y_lb', _tvp['y_min'] - _x['car_y'], ub=0) #, soft_constraint=True, penalty_term_cons=1e2)
    # mpc.set_nl_cons('y_ub', _x['car_y'] - _tvp['y_max'], ub=0) #, soft_constraint=True, penalty_term_cons=1e2)

    # Don't need to set bounds for the states
    # mpc.bounds['lower', '_x', 'car_x'] = mpc_x_min
    # mpc.bounds['lower', '_x', 'car_y'] = mpc_y_min
    # mpc.bounds['upper', '_x', 'car_x'] = mpc_x_max
    # mpc.bounds['upper', '_x', 'car_y'] = mpc_y_max
     
    return mpc
