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
    #mterm = ((_tvp['target_x']- _x['car_x'])) + ((_tvp['target_y'] - _x['car_y']))
    #lterm = casadi.DM.zeros()
    lterm = ((_tvp['target_x']- _x['car_x']) ** 2) + ((_tvp['target_y'] - _x['car_y']) ** 2)
    mterm = ((_tvp['target_x']- _x['car_x']) ** 4) + ((_tvp['target_y'] - _x['car_y']) ** 4)

    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(car_v=0.3 , car_delta=0.6)

    mpc.bounds['lower', '_u', 'car_v'] = 0.0
    mpc.bounds['lower', '_u', 'car_delta'] = -0.6189
   

    mpc.bounds['upper', '_u', 'car_v'] = 1.5
    mpc.bounds['upper', '_u', 'car_delta'] = 0.6189


    mpc.bounds['lower', '_x', 'car_x'] = mpc_x_min
    mpc.bounds['lower', '_x', 'car_y'] = mpc_y_min
    mpc.bounds['upper', '_x', 'car_x'] = mpc_x_max
    mpc.bounds['upper', '_x', 'car_y'] = mpc_y_max

    #mpc.setup()

    # mpc = do_mpc.controller.MPC(model)

    # setup_mpc = {
    #     'n_horizon': 5,
    #     'n_robust': 0,                         # Robust horizon for robust scenario-tree MPC,
    #     'open_loop': 0,
    #     't_step': 0.2,
    #     'state_discretization': 'collocation', # no other option at the moment
    #     'collocation_type': 'radau',           # no other option at the moment
    #     'collocation_deg': 2,
    #     'collocation_ni': 2,
    #     'store_full_solution': False, # re
    #     #'nlpsol_opts': {'ipopt.linear_solver': 'MA27'} # highly recommended, Use MA27 linear solver in ipopt for faster calculations
    # }
        
    # mpc.set_param(**setup_mpc)

    # # penalties or the cost function you are trying to optimize
    # mterm = (model.x['X_s']-tarx)**2 + (model.x['Y_s']-tary)**2 # terminal cost
    # lterm = (model.x['X_s']-tarx)**2 + (model.x['Y_s']-tary)**2 # stage cost

    # # mterm = fabs(model.x['X_s']-tarx) + fabs(model.x['Y_s']-tary) # terminal cost
    # # lterm = fabs(model.x['X_s']-tarx) + fabs(model.x['Y_s']-tary) # stage cost

    # # simple error without quadratic
    # # mterm = (model.x['X_s']-self.x_t) + (model.x['Y_s']-self.y_t) # stage cost
    # # lterm = (model.x['X_s']-self.x_t) + (model.x['Y_s']-self.y_t) # terminal cost

    # mpc.set_objective(mterm=mterm, lterm=lterm)

    # # input penalties
    # mpc.set_rterm(u_throttle = 0.9, u_steering = 0.02)

    # # set constraints
    # # start with these are there any others????????
    # # upper and lower bounds of the control input

    # mpc.bounds['lower', '_x', 'X_s'] = -20
    # mpc.bounds['lower', '_x', 'X_s'] = 20
    # mpc.bounds['lower', '_x', 'Y_s'] = -20
    # mpc.bounds['lower', '_x', 'Y_s'] = 20

    # mpc.bounds['lower','_u','u_throttle'] = 0.3
    # mpc.bounds['upper','_u','u_throttle'] = 0.5

    # mpc.bounds['lower','_u','u_steering'] = -0.523599
    # mpc.bounds['upper','_u','u_steering'] = 0.523599

    # # let's see if scaling helps at all
    # # mpc.scaling['_x', 'Theta_s'] = 1
    # # mpc.scaling['_x', 'X_s'] = 100
    # # mpc.scaling['_x', 'Y_s'] = 100
    # #optimizer.scaling['_x', 'Y_s'] = 2

    # mpc.setup()
        
    return mpc
