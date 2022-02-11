import matplotlib.pyplot as plt
import numpy as np
import random
import math

from scipy.optimize import minimize
from scipy import integrate

from scipy.spatial.distance import directed_hausdorff



def objective(x, a, b): # Objective function tries to simply minimize constants a,b,c in ax+by+c >= 0

    u0  = a
    v0  = np.zeros(shape=(len(u0),2))
    for i in range(len(u0)):
        v0[i] = [u0[i][0], x[0]*u0[i][0] + x[1]]   

    u1 = b
    v1 = np.zeros(shape=(len(u1),2))
    for i in range(len(u1)):
        v1[i] = [u1[i][0], x[2]*u1[i][0] + x[3]]  
        
    return np.sum((u0-v0)**2) + np.sum((u1-v1)**2) 
    
# Objective function based on Hausdorff distance - max(max(directed_hausdorff(u0, v0)[0], directed_hausdorff(v0, u0)[0]), max(directed_hausdorff(u1, v1)[0], directed_hausdorff(v1, u1)[0]))


def constraint_ego_car_hyperplane_0(i): #  hyperplane to left from the ego car

    def g(x, a, b, c, d, e, f): # c,d = ego_x, d = tarx
    # select a, b of the formula
    # Algorithm for project ego and target coordinates to the hyperplane - https://stackoverflow.com/questions/10301001/perpendicular-on-a-line-segment-from-a-given-point
    
        if (((e - d)**2 <= (c-d)**2) or ((f - d)**2 <= (c-d)**2)):
        # Find aX, aY        
            aX = d
            aY = x[0] * aX + x[1] 
        
        # Find bX, bY	
            bX = c
            bY = x[0] * bX + x[1]             
        else:
            aX = c
            aY =  x[0] * aX + x[1]     
            
            bX = d
            bY = x[0] * bX + x[1] 
            
	
	#Find cX, cY
        cX = a
        cY = b          
		
        
        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

    return g

def constraint_ego_car_hyperplane_1(i): #  hyperplane to left from the ego car

    def g(x, a, b, c, d, e, f): # c,d = ego_x, d = tarx
    # select a, b of the formula
    # Algorithm for project ego and target coordinates to the hyperplane - https://stackoverflow.com/questions/10301001/perpendicular-on-a-line-segment-from-a-given-point
    
        if (((e - d)**2 <= (c-d)**2) or ((f - d)**2 <= (c-d)**2)):
        # Find aX, aY        
            aX = c
            aY =  x[2] * aX + x[3] 
        
        # Find bX, bY	
            bX = d
            bY = x[2] * bX + x[3] 
        else:
            aX = d
            aY =  x[2] * aX + x[3]     
            
            bX = c
            bY = x[2] * bX + x[3] 
	
	#Find cX, cY
        cX = a
        cY = b          
		

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

    return g

def constraint_opp_car_hyperplane_0(i):

    def g(x, a, b, c, d, e, f): # c,d = ego_x, d = tarx
    # select a, b of the formula
    # Algorithm for project ego and target coordinates to the hyperplane - https://stackoverflow.com/questions/10301001/perpendicular-on-a-line-segment-from-a-given-point
    
        if (((e - d)**2 <= (c-d)**2) or ((f - d)**2 <= (c-d)**2)):
        # Find aX, aY        
            aX = c
            aY =  x[0] * aX + x[1] 
        
        # Find bX, bY	
            bX = d
            bY = x[0] * bX + x[1] 
        else:
            aX = d
            aY =  x[0] * aX + x[1]     
            
            bX = c
            bY = x[0] * bX + x[1] 
	
	#Find cX, cY
        cX = a
        cY = b          
		

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

     
    return g
    
def constraint_opp_car_hyperplane_1(k):

    def g(x, a, b, c, d, e, f): # c,d = ego_x, d = tarx
    # select a, b of the formula
    # Algorithm for project ego and target coordinates to the hyperplane - https://stackoverflow.com/questions/10301001/perpendicular-on-a-line-segment-from-a-given-point
    
        if (((e - d)**2 <= (c-d)**2) or ((f - d)**2 <= (c-d)**2)):
        # Find aX, aY        
            aX = d
            aY =  x[2] * aX + x[3] 
        
        # Find bX, bY	
            bX = c
            bY = x[2] * bX + x[3] 
        else:
            aX = c
            aY =  x[2] * aX + x[3]     
            
            bX = d
            bY = x[2] * bX + x[3] 
	
	#Find cX, cY
        cX = a
        cY = b          
		

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 
     
    return g




def constraint_target_hyperplane_0(k):

    def g(x, a, b, c, d, e, f): # c,d = ego_x, d = tarx
    # select a, b of the formula
    # Algorithm for project ego and target coordinates to the hyperplane - https://stackoverflow.com/questions/10301001/perpendicular-on-a-line-segment-from-a-given-point
    
        if (((e - d)**2 <= (c-d)**2) or ((f - d)**2 <= (c-d)**2)):
        # Find aX, aY        
            aX = d
            aY =  x[0] * aX + x[1] 
        
        # Find bX, bY	
            bX = c
            bY = x[0] * bX + x[1] 
        else:
            aX = c
            aY =  x[0] * aX + x[1]     
            
            bX = d
            bY = x[0] * bX + x[1] 
	
	#Find cX, cY
        cX = a
        cY = b          
		

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

     
    return g
    
def constraint_target_hyperplane_1(k):

    def g(x, a, b, c, d, e, f): # c,d = ego_x, d = tarx
    # select a, b of the formula
    # Algorithm for project ego and target coordinates to the hyperplane - https://stackoverflow.com/questions/10301001/perpendicular-on-a-line-segment-from-a-given-point
    
        if (((e - d)**2 <= (c-d)**2) or ((f - d)**2 <= (c-d)**2)):
        # Find aX, aY        
            aX = c
            aY =  x[2] * aX + x[3] 
        
        # Find bX, bY	
            bX = d
            bY = x[2] * bX + x[3] 
        else:
            aX = d
            aY =  x[2] * aX + x[3]     
            
            bX = c
            bY = x[3] * bX + x[3] 
	
	#Find cX, cY
        cX = a
        cY = b          
		

        return ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX)) 

     
    return g   


     




def find_constraints(ego_x, ego_y, head_angle, array_left, array_right, tarx, tary, ego_cc):


 # initial guesses
    n = 4
    x0 = np.zeros(n)
    b = (-100, 100)
    bnds = (b, b, b, b)
    #xp = 3
    
    angle =  head_angle * (180/math.pi)
    

    cons = []
    ar_ego =  np.array([[ego_x, ego_y], [ego_cc[0][0], ego_cc[0][1]], [ego_cc[1][0], ego_cc[1][1]], [ego_cc[2][0], ego_cc[2][1]], [ego_cc[3][0], ego_cc[3][1]], [tarx, tary]])
    ar_tar =  np.array([[tarx, tary]])

    tr_x = ego_cc[0][0]
    tl_x = ego_cc[1][0]
    for a in range(len(ar_ego)): # create ego car constraints (each corner of the car) - from the right
        arguments2 = (ar_ego[a][0], ar_ego[a][1], ego_x, tarx, tr_x, tl_x)
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_ego_car_hyperplane_0(a)})

    for b in range(len(ar_ego)): # create ego car constraints (each corner of the car) - from the left
        arguments2 = (ar_ego[b][0], ar_ego[b][1], ego_x, tarx, tr_x, tl_x)
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_ego_car_hyperplane_1(b)})        
    
    for c in range(len(array_left)): # create static obstacles constraints - obstacles from the right
        arguments2 = (array_left[c][0], array_left[c][1], ego_x, tarx, tr_x, tl_x)
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_opp_car_hyperplane_0(c)})
      
    for d in range(len(array_right)): # create static obstacles constraints - obstacles from the left
       arguments2 = (array_right[d][0], array_right[d][1], ego_x, tarx, tr_x, tl_x)   
       cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_opp_car_hyperplane_1(d)})   
         
    for e in range(len(ar_tar)): # create static obstacles constraints - obstacles from the left
        arguments2 = (ar_tar[e][0], ar_tar[e][1], ego_x, tarx, tr_x, tl_x) 
        cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_target_hyperplane_0(e)}) 
 
    for f in range(len(ar_tar)): # create static obstacles constraints - obstacles from the left
       arguments2 = (ar_tar[f][0], ar_tar[f][1], ego_x, tarx, tr_x, tl_x) 
       cons.append({'type': 'ineq', 'args': arguments2, 'fun': constraint_target_hyperplane_1(f)}) 
   
   
  
       
    solution = minimize(objective, x0, args=(array_left, array_right), method='COBYLA', constraints=cons)
    x = solution.x

    return x 
    
 
  


