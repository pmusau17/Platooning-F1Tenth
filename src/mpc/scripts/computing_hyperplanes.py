import matplotlib.pyplot as plt
import numpy as np

from casadi import *
from casadi.tools import *
from scipy.optimize import minimize


ar_b = np.array([[2, 2], [3, 2], [2.2, 2.5], [3.2, 2.5], [2.4, 3], [3.4, 3]])


ar_u = np.array([[-1, 2], [-0.9, 2], [-.8, 2.5], [-.7, 2.5], [-.6, 3], [-.5, 3], [-.4, 3], [-.3, 3], [-.2, 3], [-.1, 3]])

def objective(x, xp): # Objective function tries to simply minimize constants a,b,c in ax+by+c >= 0

    return  abs(x[0] + x[1] + x[2] + x[3])


def constraint_ego_car_bottom(x, a, b): #constraint that car coordinates (x_c, y_c) >= ax + by +c >= 0

    return b - (x[0] * a + x[1]) - .4
    
def constraint_ego_car_upper(x, a, b): #constraint that car coordinates (x_c, y_c) >= ax + by +c >= 0

    return (x[2] * a + x[3]) - b  


def constraint_opp_car_bottom(x): 
      
    i = 0
    bottom_value = 0
    while i < len(ar_b):
        if ((x[0]* ar_b[i][0] + x[1]) - ar_b[i][1] < 0):
            bottom_value =  (x[0]* ar_b[i][0] + x[1]) - ar_b[i][1]
            i = len(ar_b)
        else:
            bottom_value += (x[0]* ar_b[i][0] + x[1]) - ar_b[i][1] 
            i += 1    
    
    return bottom_value
    
def constraint_opp_car_upper(x):  
    
    i = 0
    upper_value = 0
    while i < len(ar_u):
        if (ar_u[i][1] - (x[2]* ar_u[i][0] + x[3]) < 0):
            upper_value=  ar_u[i][1] - (x[2]* ar_u[i][0] + x[3])
            print(ar_u[i][0], ar_u[i][1], ar_u[i][1] - (x[2]* ar_u[i][0] + x[3]), x[2], x[3])
            i = len(ar_u)
        else:
            upper_value += ar_u[i][1] - (x[2]* ar_u[i][0] + x[3]) 
            print(ar_u[i][0], ar_u[i][1], ar_u[i][1] - (x[2]* ar_u[i][0] + x[3]) )
            i += 1    
    print(upper_value)
    return upper_value

def find_constraints(ego_x, ego_y):
    x_car = ego_x
    y_car = ego_y


# initial guesses
    n = 4
    x0 = np.zeros(n)
    x0[0] = 1.0
    x0[1] = 5.0
    x0[2] = 5.0
    x0[3] = 5.0
    
   
    

    arguments = (x_car, y_car)
    #arguments2 = arg
    
 

# optimize
    b = (-np.inf, np.inf)
    bnds = (b, b, b, b)
    xp = 3
    
    con0 = {'type': 'ineq', 'args': arguments, 'fun': constraint_ego_car_bottom}
    con1 = {'type': 'ineq', 'args': arguments, 'fun': constraint_ego_car_upper}  
    con2 = {'type': 'ineq',  'fun': constraint_opp_car_bottom}
    con3 = {'type': 'ineq',  'fun': constraint_opp_car_upper}

    cons = ([con0, con1, con2, con3]) 
    solution = minimize(objective, x0, args=(xp), method='SLSQP', bounds=bnds, constraints=cons)
    x = solution.x

    return x 
    
    
if __name__ == "__main__":

    # This is the starting x, and y position of the ego vehicle 
    ex = 0
    ey = 1
   
      

    # a_b, b_b, a_u, b_u = find_constraints(ex, ey)
    # check = abs(1 - abs(a_b * 2 + b_b + a_u * 2 + b_u))
    # print(a_b, b_b, a_u, b_u, check)
  
    # x = np.linspace(-3, 3)
    # y_b = a_b * x + b_b
    # y_u =  a_u * x + b_u
    
    # plot the points characterizing the points in this example, these are stored in ar_u
    x1 = ar_u[:,0] # scatter plot
    y2 = ar_u[:,1] # scatter plot
    plt.scatter(x1, y2) # scatter plot
    
  
    # plt.plot(x, y_b)
    # plt.plot(x, y_u)

    plt.plot(ex, ey, 'ro') 
    # plt.plot([2, 3], [2, 2])
    # plt.plot([2.2, 3.2], [2.5, 2.5])
    # plt.plot([2.4, 3.4,], [3, 3])
    plt.show()
  

