import matplotlib.pyplot as plt
import numpy as np
import random
import math


from scipy.optimize import minimize




def objective(x): # Objective function tries to simply minimize constants a,b,c in ax+by+c >= 0

    return 1


def constraint_ego_car_hyperplane_0(i): #  hyperplane to left from the ego car

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

def constraint_target_hyperplane_0(x, a, b): 


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
     

def constraint_target_hyperplane_1(x, a, b): 


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


def constraint_dynamic_obstacles_hyperplane_0(i):

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


def constraint_dynamic_obstacles_hyperplane_1(k):

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



def find_constraints(ego_x, ego_y, array_left, array_right, tarx, tary):


 # initial guesses
    n = 4
    x0 = np.zeros(n)
    b = (-100, 100)
    bnds = (b, b, b, b)
    #xp = 3
    


    cons = []
    ar_ego =  np.array([[ego_x, ego_y], [ego_x, ego_y + 0.4], [ego_x + .24, ego_y], [ego_x + .24, ego_y + 0.4]])
    ar_u = array_left
    ar_b = array_right
    ar_d_l = array_left
    ar_d_r = array_right
    
    for a in range(len(ar_ego)): # create ego car constraints (each corner of the car) - from the right
        arguments2 = (ar_ego[a][0], ar_ego[a][1])
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_ego_car_hyperplane_0(a)})

    for b in range(len(ar_ego)): # create ego car constraints (each corner of the car) - from the left
        arguments2 = (ar_ego[b][0], ar_ego[b][1])
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_ego_car_hyperplane_1(b)})        
    
    for c in range(len(ar_u)): # create static obstacles constraints - obstacles from the right
        arguments2 = (ar_u[c][0], ar_u[c][1])
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_opp_car_hyperplane_0(c)})
      
    for d in range(len(ar_b)): # create static obstacles constraints - obstacles from the left
       arguments2 = (ar_b[d][0], ar_b[d][1])
       cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_opp_car_hyperplane_1(d)})     
     
    cons.append({'type': 'ineq', 'args': (tarx, tary), 'fun': constraint_target_hyperplane_0})  
    
    cons.append({'type': 'ineq', 'args': (tarx, tary), 'fun': constraint_target_hyperplane_1})   
            
        
    
    solution = minimize(objective, x0, method='SLSQP', bounds=bnds, constraints=cons)
    x = solution.x

    return x 
    
