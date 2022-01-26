import matplotlib.pyplot as plt
import numpy as np
import random
import math

from scipy.optimize import minimize




def objective(x): # Objective function tries to simply minimize constants a,b,c in ax+by+c >= 0


    return 1


def constraint_ego_car_hyperplane_0(i): #  hyperplane to left from the ego car

    def g(x, a, b, c, d): # c,d = ego_x, ego_y
    # select a, b of the formula
    # Algorithm for project ego and target coordinates to the hyperplane - https://stackoverflow.com/questions/10301001/perpendicular-on-a-line-segment-from-a-given-point
    
          
        # Find aX, aY        
        aX = d
        aY =  x[0] * aX + x[1] 
        
        # Find bX, bY	
        bX = c
        bY = x[0] * bX + x[1] 
	
	#Find cX, cY
        cX = a
        cY = b          
		

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

    return g

def constraint_ego_car_hyperplane_1(i): #  hyperplane to left from the ego car

    def g(x, a, b, c, d):
    # select a, b of the formula
                
        # Find aX, aY
        
        aX = c
        aY = x[2] * aX + x[3] 

	    
        # Find bX, bY
        bX = d
        bY = x[2] * bX + x[3] 
		
        cX = a
        cY = b          
		 

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

    return g

def constraint_opp_car_hyperplane_0(i):

    def g(x, a, b, c, d): # c,d = ego_x, ego_y
    # select a, b of the formula
    # Algorithm for project ego and target coordinates to the hyperplane - https://stackoverflow.com/questions/10301001/perpendicular-on-a-line-segment-from-a-given-point
    
          
        # Find aX, aY        
        aX = c
        aY =  x[0] * aX + x[1] 
        
        # Find bX, bY	
        bX = d
        bY = x[0] * bX + x[1] 
	
	#Find cX, cY
        cX = a
        cY = b          
		

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

     
    return g
    
def constraint_opp_car_hyperplane_1(k):

    def g(x, a, b, c, d):
    # select a, b of the formula
                
        # Find aX, aY
        
        aX = d
        aY = x[2] * aX + x[3] 

	    
        # Find bX, bY
        bX = c
        bY = x[2] * bX + x[3] 
		
        cX = a
        cY = b          
		 

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 
     
    return g




def constraint_target_hyperplane_0(k):

    def g(x, a, b, c, d): # c,d = ego_x, ego_y
    # select a, b of the formula
    # Algorithm for project ego and target coordinates to the hyperplane - https://stackoverflow.com/questions/10301001/perpendicular-on-a-line-segment-from-a-given-point
    
          
        # Find aX, aY        
        aX = d
        aY =  x[0] * aX + x[1] 
        
        # Find bX, bY	
        bX = c
        bY = x[0] * bX + x[1] 
	
	#Find cX, cY
        cX = a
        cY = b          
		

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

     
    return g
    
def constraint_target_hyperplane_1(k):

    def g(x, a, b, c, d):
    # select a, b of the formula
                
        # Find aX, aY
        
        aX = c
        aY = x[2] * aX + x[3] 

	    
        # Find bX, bY
        bX = d
        bY = x[2] * bX + x[3] 
		
        cX = a
        cY = b          
		 

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

     
    return g   


     




def find_constraints(ego_x, ego_y, head_angle, array_left, array_right, tarx, tary):


 # initial guesses
    n = 4
    x0 = np.zeros(n)
    b = (-100, 100)
    bnds = (b, b, b, b)
    #xp = 3
    
    angle =  head_angle * (180/math.pi)
    
    tr_x= ego_x + ((0.203 / 2) * math.cos(angle)) - ((0.15875) * math.sin(angle))
    tr_y = ego_y + ((0.203  / 2) * math.sin(angle)) + ((0.15875) * math.cos(angle))


    tl_x = ego_x - ((0.203  / 2) * math.cos(angle)) - ((0.15875) * math.sin(angle))
    tl_y = ego_y - ((0.203  / 2) * math.sin(angle)) + ((0.15875) * math.cos(angle))

    bl_x = ego_x - ((0.203  / 2) * math.cos(angle)) + ((0.17145) * math.sin(angle))
    bl_y = ego_y - ((0.203  / 2) * math.sin(angle)) - ((0.17145) * math.cos(angle))

    br_x = ego_x + ((0.203  / 2) * math.cos(angle)) + ((0.17145) * math.sin(angle))
    br_y = ego_y + ((0.203  / 2) * math.sin(angle)) - ((0.17145) * math.cos(angle))

    cons = []
    ar_ego =  np.array([[tr_x, tr_y], [tl_x, tl_y], [bl_x, bl_y], [br_x, br_y]])
    ar_tar =  np.array([[tarx, tary], [tarx, tary], [tarx, tary], [tarx, tary]])


    
    for a in range(len(ar_ego)): # create ego car constraints (each corner of the car) - from the right
        arguments2 = (ar_ego[a][0], ar_ego[a][1], ego_x, tarx)
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_ego_car_hyperplane_0(a)})

    for b in range(len(ar_ego)): # create ego car constraints (each corner of the car) - from the left
        arguments2 = (ar_ego[b][0], ar_ego[b][1], ego_x, tarx)
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_ego_car_hyperplane_1(b)})        
    
    for c in range(len(array_left)): # create static obstacles constraints - obstacles from the right
        arguments2 = (array_left[c][0], array_left[c][1], ego_x, tarx)
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_opp_car_hyperplane_0(c)})
      
    for d in range(len(array_right)): # create static obstacles constraints - obstacles from the left
       arguments2 = (array_right[d][0], array_right[d][1], ego_x, tarx)   
       cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_opp_car_hyperplane_1(d)})   
         
    for e in range(len(ar_tar)): # create static obstacles constraints - obstacles from the left
        arguments2 = (ar_tar[e][0], ar_tar[e][1], ego_x, tarx) 
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_target_hyperplane_0(e)}) 
 
    for f in range(len(ar_tar)): # create static obstacles constraints - obstacles from the left
       arguments2 = (ar_tar[f][0], ar_tar[f][1], ego_x, tarx) 
       cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_target_hyperplane_1(f)}) 
   
   
  
       
    solution = minimize(objective, x0, method='SLSQP', bounds=None, constraints=cons)
    x = solution.x

    return x 
    
 
  


