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
        't_step': 0.1,
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
    _u = model.u

    # change the objective function to a time varying parameter

    #lterm = DM.zeros()
    #lterm = ((_tvp['target_x']- _x['car_x']) ** 2) + ((_tvp['target_y'] - _x['car_y']) ** 2)
    #lterm = ((_tvp['target_x']- _x['car_x']) ** 2) + ((_tvp['target_y'] - _x['car_y']) ** 2)  + (_x["time"])**2

    #lterm = (_x["time"])**2

    # I've tried penalizing theta but it might be that the model theta error is quite large since
    # it's a linear model ((_tvp['target_theta'] - _x['car_theta']) ** 2)
    
    # objective function of euclidean distance in xyz
    mterm = ((_tvp['target_x']- _x['car_x']) ** 2) + ((_tvp['target_y'] - _x['car_y']) ** 2) 
    +  ((_tvp['target_theta'] - _x['car_theta']) ** 2)# +  (_x["time"])**2

    lterm = mterm - _u['car_v']
    
    mpc.set_objective(mterm=mterm, lterm=lterm)
    # configurations that I've tried
    # 2.0, 2.0 (pretty smooth, delayed turning)
    # mpc.set_rterm(car_v=0.0 , car_delta=1.0)

    # the r_term is quite sensitive
    # use this one if the speed is over 2
    #mpc.set_rterm(car_v=15.0 , car_delta=0.0)
    mpc.set_rterm(car_v=1.0 , car_delta=0.5)

    mpc.bounds['lower', '_u', 'car_v'] = 0.0
    mpc.bounds['lower', '_u', 'car_delta'] = -0.6189
    mpc.bounds['upper', '_u', 'car_v'] = 1.5
    mpc.bounds['upper', '_u', 'car_delta'] = 0.6189

    #mpc.scaling['_x', 'car_theta'] = 2
    #mpc.scaling['_x', 'car_x'] = 2
    #mpc.scaling['_x', 'car_y'] = 2

    



    # left plane constraint, this basically bounds it to the right or to the left
    mpc.set_nl_cons('constraint_left_plane',  _tvp['c1'] * ((_tvp['a0'] * _x['car_x'] + _tvp['b0']) - _x['car_y']), 0)

    # left plane constraint, this basically bounds it to the right or to the left
    mpc.set_nl_cons('constraint_right_plane', _tvp['c2'] * ((_tvp['a1'] * _x['car_x'] + _tvp['b1']) - _x['car_y']), 0)


    # constraints to remain outside of opponent hyper_rectangle I think this makes the search space nonconvex....
    mpc.set_nl_cons('x_min_cons',  _x['car_x']-_tvp['x_min'], 0)
    mpc.set_nl_cons('x_max_cons',  _tvp['x_max']-_x['car_x'], 0)
    mpc.set_nl_cons('y_min_cons',  _x['car_y']-_tvp['y_min'], 0)
    mpc.set_nl_cons('y_max_cons',  _tvp['y_max']-_x['car_y'], 0)


 
    return mpc
