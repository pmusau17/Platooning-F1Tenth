// Patrick Musau
// 08-2020
// Safety checking for uuv model file

#include "bicycle_dynamic_safety.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 
#include <stdlib.h>
#include <sys/time.h>



// provide initial value for global variables
double *** obstacles = 0;
int file_rows = 0;
int file_columns = 0;
int obstacle_count = 0;

bool check_safety(HyperRectangle* rect, REAL (*cone)[2])
{
	
	REAL l1[2] = {rect->dims[0].min,rect->dims[1].max};
    REAL r1[2] = {rect->dims[0].max,rect->dims[1].min};

	REAL l2[2] = {cone[0][0],cone[1][1]};
    REAL r2[2] = {cone[0][1],cone[1][0]};

	if (l1[0] >= r2[0] || l2[0] >= r1[0])
    {
        return true; 
    }
    
    if (l1[1] <= r2[1] || l2[1] <= r1[1])
    {
        return true;
    }

	return false;
}


bool check_safety_obstacles(HyperRectangle* rect)
{
    // loop through the obstacles
    bool allowed = true;
	for (int j = 0; j < obstacle_count; j++)
	{	
        double obs[2][2]  = {{obstacles[j][0][0],obstacles[j][0][1]}, {obstacles[j][1][0],obstacles[j][1][1]}};
        allowed= check_safety(rect,obs);
        if(!allowed)
        {   
            // printf("offending cone [%f, %f], ,[%f, %f]\n",obstacles[j][0][0],obstacles[j][0][1],obstacles[j][1][0],obstacles[j][1][1]);
            break;
        }
	}

    return allowed;
}

// function that allocates the 3d array for the obstacles
void allocate_obstacles(int num_obstacles)
{

    int rows = num_obstacles;
    int cols = 2;
    int height = 2;
    int i,j;

    obstacles = (double***)malloc(rows * sizeof(double **));
    
    // check if memory was allocated 
    if(obstacles == NULL)
	{
		fprintf(stderr, "out of memory\n");
		exit(0);
	}

    for(i=0;i<rows;i++)
    {
        obstacles[i] = (double **)malloc(cols * sizeof(double*));
        // check if memory was allocated
        if(obstacles[i] == NULL)
		{
			fprintf(stderr, "out of memory\n");
			exit(0);
		}
        for(j=0;j<cols;j++)
        {
            obstacles[i][j] = (double*)malloc(height*sizeof(double));
            // check if memory was allocated
            if(obstacles[i][j] == NULL)
		    {
			    fprintf(stderr, "out of memory\n");
			    exit(0);
		    }
        }
    }
}

void print_obstacles()
{
    printf("interval list of obstacles: \n");
    for(int i=0;i<obstacle_count;i++)
    {
        printf("[%f,%f], [%f,%f]\n", obstacles[i][0][0],obstacles[i][0][1],obstacles[i][1][0],obstacles[i][1][1]);
    }
    printf("\n");
}

void append_obstacle(int index,double (*box)[2])
{
    obstacles[index][0][0] = box[0][0];
    obstacles[index][0][1] = box[0][1];
    obstacles[index][1][0] = box[1][0];
    obstacles[index][1][1] = box[1][1];
}

// free the memory allocated for the wall points
void deallocate_obstacles()
{
    int rows = obstacle_count;
    int cols = 2;
    int i,j;
    for(i=0;i<rows;i++)
    {
        for(j=0;j<cols;j++)
        {
            free(obstacles[i][j]);
        }
        free(obstacles[i]);
    }
    free(obstacles);
}
