import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc

def template_model():

    # either 'discrete' or 'continuous'
    model_type = "discrete"
    model = do_mpc.model.Model(model_type)

    car_x = model.set_variable(var_type='_x', var_name='car_x', shape=(1, 1))
    car_y = model.set_variable(var_type='_x', var_name='car_y', shape=(1, 1))
    car_velocity = model.set_variable(var_type='_x', var_name='car_velocity', shape=(1, 1))
    car_theta = model.set_variable(var_type='_x', var_name='car_theta', shape=(1, 1))

    car_v = model.set_variable(var_type='_u', var_name='car_v')
    car_delta = model.set_variable(var_type='_u', var_name='car_delta')

  
    model.set_rhs("car_x",  (-0.0059180668130126) * car_x + 0.0302298695844774 * car_y + 1.7340331227345 * car_velocity +  (-0.0297223204117808) * car_theta + (-3.38382631868513) * car_v + (0.926366053232582) * car_delta)
    model.set_rhs("car_y", (-0.0811606340155255) * car_x +  0.00122828961313501 * car_y + (-15.6910154905913) * car_velocity +  (1.22769534659242) * car_theta +   31.1666962413352 * car_v + (-2.0448587148705) * car_delta)
    model.set_rhs("car_velocity", (-0.0010819205320157) * car_x + (-0.000105319455475052) * car_y + (-0.249757277582323) * car_velocity + (-0.00180539229455585) * car_theta + 0.495766762277423 * car_v + (0.0451325222147371) * car_delta)
    model.set_rhs("car_theta",   (-0.0354224993672635) * car_x + (0.00519584252812829) * car_y + (-7.030615315487) * car_velocity +  (-0.312454355668216) * car_theta + 13.9957622403336 * car_v + (2.24774201891125) * car_delta)
    
#     model.set_rhs("car_x",  (229974865091351*car_v)/144115188075855872 + (1126839741810155*car_x)/1125899906842624 + (1054953793716431*car_y)/1152921504606846976 + (5007684948557881*car_delta)/144115188075855872 + (2264038713151901*car_theta)/288230376151711744 + (3069901934028197*car_velocity)/576460752303423488)
#     model.set_rhs("car_y", (4503902775611039*car_y)/4503599627370496 - (8155328636645421*car_x)/2305843009213693952 - (2185042808221847*car_v)/576460752303423488 - (4205827695942601*car_delta)/72057594037927936 + (83416275937563*car_theta)/2251799813685248 - (1644007457666287*car_velocity)/144115188075855872)
#     model.set_rhs("car_velocity", (1970980323083*car_v)/2199023255552 - (171797914204025*car_x)/144115188075855872 + (6316030347926583*car_y)/295147905179352825856 - (6719695863384683*car_delta)/144115188075855872 - (3166932523453749*car_theta)/2305843009213693952 + (1248405412772095*car_velocity)/2251799813685248)
#     model.set_rhs("car_theta", (1236130123132999*car_v)/72057594037927936 - (1030232773200503*car_x)/288230376151711744 - (1082749374786055*car_y)/9223372036854775808 - (810182633622651*car_delta)/36028797018963968 + (2204646398000493*car_theta)/2251799813685248 - (3617580828719669*car_velocity)/144115188075855872)
    
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

    # Create time-varying-parameters, these will be populated with (potentially) different data at each call
    model.set_variable(
            var_type="_tvp",
            var_name="speed_min",
            shape=(1, 1)
    )
    model.set_variable(
            var_type="_tvp",
            var_name="speed_max",
            shape=(1, 1)
    )


    # Build the model
    model.setup()

    return model

if __name__ == "__main__":
    template_model()