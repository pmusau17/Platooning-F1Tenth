# import the necessary libraries

import matplotlib.pyplot as plt
import numpy as np
import random
import math

from scipy.optimize import minimize


def load_wall_points():
    x_s = []
    y_s  = []
    with open("porto_obstacles.txt") as f:
        content = f.read().split("\n")
        for point in content:
            if(point):
                x_s.append(float(point.split(",")[0]))
                y_s.append(float(point.split(",")[1]))
    x_s = np.asarray(x_s)
    y_s = np.asarray(y_s)
    return x_s, y_s

def compute_points_near_vehicle(ex,ey,x_s,y_s):

    pts = np.vstack((x_s,y_s)).T
    plt.scatter(pts[:,0],pts[:,1])
    eg = np.asarray([ex,ey]).reshape((-1,2))
    pt_diff = pts - eg 
    norm = np.linalg.norm(pt_diff,axis=-1)
    indices = np.where(norm<=2)[0]
    new_pts = pts[indices]
    new_x_s = new_pts[:,0]
    new_y_s = new_pts[:,1]
    return new_x_s, new_y_s




def objective(x, xp): 

    """
        Objective function tries to simply minimize constants a,b,c in ax+by+c >= 0
    """

    return  (x[1]) - (x[0]) 


def constraint_ego_car_hyperplane_0(i): 
    
    
    """
    hyperplane to left from the ego car

    """

    def g(x, a, b):
    # select a, b of the formula
        if (x[0] > 0):        
            aX = 0
            aY = x[1]

	    
            bX = aX - 1
            bY = x[0] * bX + x[1] 
		
            cX = a
            cY =  b          
		
        else:
            aX = 0
            aY = x[1]
	    
            bX = aX + 1
            bY = x[0] * bX + x[1]    

            cX = a
            cY =  b  

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

    return g


def constraint_ego_car_hyperplane_1(i): 

    """
        hyperplane to left from the ego car
    """
    
    def g(x, a, b):
    # select a, b of the formula
        if (x[2] > 0):        
            aX = 0
            aY = x[3]

	    
            bX = aX + 1
            bY = x[2] * bX + x[3] 
		
            cX = a
            cY =  b          
		
        else:
            aX = 0
            aY = x[3]
	    
            bX = aX - 1
            bY = x[2] * bX + x[3]    

            cX = a
            cY =  b  

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

    return g


def constraint_opp_car_hyperplane_0(i):

    def g(x, a, b):
    
        if (x[0] > 0):        
            aX = 0
            aY = x[1]
    
            bX = aX + 1
            bY = x[0] * bX + x[1] 
        
            cX = a
            cY =  b          
        
        else:
            aX = 0
            aY = x[1]
    
            bX = aX - 1
            bY = x[0] * bX + x[1]    

            cX = a
            cY =  b  
    
    
        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX))  

     
    return g


def constraint_ego_car_hyperplane_1(i): #  hyperplane to left from the ego car

    def g(x, a, b):
    # select a, b of the formula
        if (x[2] > 0):        
            aX = 0
            aY = x[3]

	    
            bX = aX + 1
            bY = x[2] * bX + x[3] 
		
            cX = a
            cY =  b          
		
        else:
            aX = 0
            aY = x[3]
	    
            bX = aX - 1
            bY = x[2] * bX + x[3]    

            cX = a
            cY =  b  

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

    return g


def constraint_opp_car_hyperplane_0(i):

    def g(x, a, b):
    
        if (x[0] > 0):        
            aX = 0
            aY = x[1]
    
            bX = aX + 1
            bY = x[0] * bX + x[1] 
        
            cX = a
            cY =  b          
        
        else:
            aX = 0
            aY = x[1]
    
            bX = aX - 1
            bY = x[0] * bX + x[1]    

            cX = a
            cY =  b  
    
    
        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX))  

     
    return g
    
def constraint_opp_car_hyperplane_1(k):

    def f(x, a, b):
    
        if (x[2] > 0):        
            aX = 0
            aY = x[3]
    
            bX = aX - 1
            bY = x[2] * bX + x[3] 
        
            cX = a
            cY =  b          
        
        else:
            aX = 0
            aY = x[2]
    
            bX = aX + 1
            bY = x[2] * bX + x[3]    

            cX = a
            cY =  b  
    
    
        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX))  

     
    return f



def find_constraints(ego_x, ego_y, array_left, array_right):


    # initial guesses
    n = 4
    x0 = np.zeros(n)
    #b = (-100, 100)
    #bnds = (b, b)
    xp = 3
    
    cons = []
    ar_ego =  np.array([[ego_x, ego_y], [ego_x, ego_y + 0.4], [ego_x + .24, ego_y], [ego_x + .24, ego_x + 0.4]])
    ar_u = array_left
    ar_b = array_right
    
    for a in range(len(ar_ego)):
        arguments2 = (ar_ego[a][0], ar_ego[a][1])
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_ego_car_hyperplane_0(a)})

    for b in range(len(ar_ego)):
        arguments2 = (ar_ego[b][0], ar_ego[b][1])
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_ego_car_hyperplane_1(b)})        
    
    for i in range(len(ar_u)):
        arguments2 = (ar_u[i][0], ar_u[i][1])
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_opp_car_hyperplane_0(i)})
    
    
    for k in range(len(ar_b)):
       arguments2 = (ar_b[k][0], ar_b[k][1])
       cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_opp_car_hyperplane_1(k)})     
        
    
    solution = minimize(objective, x0, args=(xp), method='SLSQP', bounds=None, constraints=cons)
    x = solution.x

    return x 




    


if __name__ == "__main__":
    
    # This is the ego starting position (ex,ey)
    ex = 0
    ey = 0
    
    x_s, y_s  = load_wall_points()
    x_n, y_n  = compute_points_near_vehicle(ex,ey,x_s,y_s)

    plt.scatter(x_s,y_s)
    plt.scatter(x_n,y_n)
    plt.show()
    
    
    #print(Solution().compute_position(content))


    
    # rw_inner = np.zeros(shape=(100,2))
    # rw_outter = np.zeros(shape=(100,2))

    # for k in range(100):
    #     rw_inner[k] = [k, pow(k, 2)/100 + round(random.uniform(5,2.9), 2)] #
    # for k in range(100):
    #     rw_outter[k] = [k, pow(k, 2)/100 + round(random.uniform(-2,-1), 2)] #
    
    # print(rw_inner,rw_outter)
    # plt.scatter(rw_inner,rw_outter)
    # plt.show()
    # rw_inner_observable_list = []
    # for k in range(100):
    #     if ((pow((rw_inner[k][0] - ex), 2) +  pow((rw_inner[k][0] - ey), 2)) <= 100):
    #         rw_inner_observable_list.append([rw_inner[k][0], rw_inner[k][1]])         
    # rw_inner_observable_array = np.array(rw_inner_observable_list)

    
    # x_inner = rw_inner_observable_array[:,0] # scatter plot
    # y_inner = rw_inner_observable_array[:,1] # scatter plot
    # plt.scatter(x_inner, y_inner) # scatter plot

    # rw_outter_observable_list = []
    # for k in range(100):
    #     if ((pow((rw_outter[k][0] - ex), 2) +  pow((rw_outter[k][0] - ey), 2)) <= 100):
    #         rw_outter_observable_list.append([rw_outter[k][0], rw_outter[k][1]])       
    
    # rw_outter_observable_array = np.array(rw_outter_observable_list)
    # x_outter = rw_outter_observable_array[:,0] # scatter plot
    # y_outter = rw_outter_observable_array[:,1] # scatter plot
    # plt.scatter(x_outter, y_outter) # scatter plot
    # plt.show()