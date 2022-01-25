import numpy as np 
import matplotlib.pyplot as plt

"""
This is where I got this procedure but essentially it's just some vector math

"""

def isinbox(box,inside=True):

    bottom_left_corner = np.asarray([box[0][0],box[1][0]])
    bottom_right_corner = np.asarray([box[0][1],box[1][0]])
    top_left_corner = np.asarray([box[0][0],box[1][1]])
    top_right_corner = np.asarray([box[0][1],box[1][1]])

    points = [bottom_left_corner, bottom_right_corner, 
            top_left_corner, top_right_corner]

    colors = ['bo','ro','go','ko']
    for i in range(len(colors)):

        plt.plot(points[i][0],points[i][1],colors[i])
    
    visited = []
    for i in range(1):

        point = [np.random.randint(box[0][0]-10,box[0][1]+10),np.random.randint(box[1][0]-10,box[1][1]+10)]
        if(point in visited):
            continue
        p1 = np.asarray(point)
        v1 = (top_left_corner-bottom_left_corner)
        v2 = (bottom_right_corner-bottom_left_corner)
        v3  = (p1 - bottom_left_corner)

        if(inside):
            # this to check for all the points with in the rectangle
            eq1 =   (np.dot(v1,bottom_left_corner) <np.dot(v1,p1))  and (np.dot(v1,p1)< np.dot(v1,top_left_corner))
            eq2 =   (np.dot(v2,bottom_left_corner) <np.dot(v2,p1)) and (np.dot(v2,p1)<np.dot(v2,bottom_right_corner))
            if(eq1 and eq2):
                plt.plot(point[0],point[1],'m.')
            else:
                plt.plot(point[0],point[1],'k.')
        else:
            # now what I need is the negation of the above formula
            eq1 =   (np.dot(v1,bottom_left_corner) > np.dot(v1,p1))  or (np.dot(v1,p1)> np.dot(v1,top_left_corner))
            eq2 =   (np.dot(v2,bottom_left_corner) > np.dot(v2,p1)) or (np.dot(v2,p1)>np.dot(v2,bottom_right_corner))

            if(eq1 or eq2):
                plt.plot(point[0],point[1],'m.')
            else:
                plt.plot(point[0],point[1],'k.')
        visited.append(point)

    plt.xlim([-20, 20])
    plt.ylim([-20, 20])
    plt.show()

if __name__ == "__main__":
    box = [[1,4],[1,4]]
    isinbox(box)


