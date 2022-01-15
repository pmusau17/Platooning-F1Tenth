from template_model import template_model
from template_mpc import template_mpc
import numpy as np 

horizon = 30 
tar_x = 1.0  
tar_y = 1.0

# this function is for changing the target position without having to 
# reframe the mpc problem
def change_target_position_template(_):
    """
    Following the docs of do_mpc, an approach to populate the target position variables with values, at any given \
    point.
    """
    template = mpc.get_tvp_template()
    print("Change_Target")
    for k in range(horizon + 1):
        template["_tvp", k, "target_x"] = 2.0
        template["_tvp", k, "target_y"] = 2.0

    return template


if __name__ ==  "__main__":


    x0 = np.asarray([0.0,0.0,0.0]).reshape((3,1))

    model =  template_model()
     # set up the mpc controller
    mpc = template_mpc(model, 30, -20, -20, 20, 20)  

        # set up the time varying function for mpc
    mpc.set_tvp_fun(change_target_position_template)
    mpc.setup()

    mpc.x0 = x0
    mpc.set_initial_guess()

    
    # print(mpc.opt_p)
    # for i in range(2):
    u0 = mpc.make_step(x0)
    print(u0)

    mpc.bounds['lower', '_x', 'car_x'] = 0
    mpc.bounds['lower', '_x', 'car_y'] = 0
    mpc.bounds['upper', '_x', 'car_x'] = 0
    mpc.bounds['upper', '_x', 'car_y'] = 0
    print(mpc.bounds['lower', '_x', 'car_x'],mpc.bounds['upper', '_x', 'car_x'])
    mpc.setup()
    u0 = mpc.make_step(x0)
    print(u0)