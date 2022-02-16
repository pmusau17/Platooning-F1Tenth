// Patrick Musau
// 08-2020
// Safety checking for f1tenth model file

#include "bicycle_safety.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 
#include <stdlib.h>
#include <sys/time.h>



// provide initial value for global variables
double ** wallCoords = 0;
double *** obstacles = 0;
int file_rows = 0;
int file_columns = 0;
int obstacle_count = 0;


// function that allocates the 2d array of wall points
void allocate_2darr(int rows,int columns)
{

    wallCoords = malloc(rows * sizeof(double *));
    if(wallCoords == NULL)
	{
		fprintf(stderr, "out of memory\n");
		exit(0);
	}
    // allocate each of the rows with arrays of length 2
    for(int i = 0; i < rows; i++)
	{
		wallCoords[i] = malloc(columns * sizeof(double));
		if(wallCoords[i] == NULL)
		{
			fprintf(stderr, "out of memory\n");
			exit(0);
		}
	}
}

// free the memory allocated for the wall points
void deallocate_2darr(int rows,int columns)
{
    for(int i = 0; i < rows; i++)
		free(wallCoords[i]);
    free(wallCoords);
    printf("Done\n");
}


int countlines(const char * filename)
{
    int cnt =0;
    FILE *fp;
    char line[60];

    // open the file
    fp = fopen(filename,"r");
    if(fp==NULL)
    {
        printf("Could not open file %s",filename);
    }
    else
    {
        while( fgets(line, 60, fp)!=NULL )
        {
            cnt+=1;
        }
        fclose(fp);
    }
    return cnt;
}

// function that loads points of the wall from the file
void load_wallpoints(const char * filename, bool print)
{
    char line[60]; 
    char * x, * y; 
    double xd, yd;
    int i;
    FILE *wallPoints;
    file_rows = countlines(filename);

    if(print)
        printf("Opening file...with %d points\n", file_rows);
    // allocate the memory
    allocate_2darr(file_rows,file_columns);
    
    // open the file
    wallPoints = fopen(filename,"r");
    if(wallPoints==NULL)
    {
        printf("Could not open file %s\n",filename);
    }
    else
    {
        i =0;
        while( fgets (line, 60, wallPoints)!=NULL )
        {
            x = strtok(line,",");
            y = strtok(NULL,",");
            xd = strtod(x,NULL);
            yd = strtod(y,NULL);
            wallCoords[i][0] = xd;
            wallCoords[i][1] = yd;
            i+=1;

        }
        fclose(wallPoints);
    }   
}




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


bool check_safety_wall(HyperRectangle* rect)
{
    bool safe = true; 
    for (int i = 0;i<file_rows;i++)
    {
        double point[2][2] = {{wallCoords[i][0],wallCoords[i][0]},{wallCoords[i][1],wallCoords[i][1]}};
        safe = check_safety(rect,point);
        if(!safe)
        {
            // printf("offending point (%f,%f)\n",wallCoords[i][0],wallCoords[i][1]);
            // println(rect);
            break;
        }
    }
    return safe;
}


// function that allocates the 3d array for the obstacles
void allocate_obstacles(int num_obstacles,double (*points)[2])
{

    int rows = num_obstacles;
    obstacle_count = num_obstacles;
    int cols = 2;
    int height = 2;
    int i,j;
    double w = 0.13;
    double h = 0.13;

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

    printf("interval list of obstacles: \n");
    for(i=0;i<rows;i++)
    {
        obstacles[i][0][0] = points[i][0]-w/2.0;
        obstacles[i][0][1] = points[i][0]+w/2.0;
        obstacles[i][1][0] = points[i][1]-h/2.0;
        obstacles[i][1][1] = points[i][1]+h/2.0;
        printf("[%f,%f], [%f,%f]\n", obstacles[i][0][0],obstacles[i][0][1],obstacles[i][1][0],obstacles[i][1][1]);
    }
    printf("\n");
}

// free the memory allocated for the wall points
void deallocate_obstacles(int num_obstacles)
{
    int rows = num_obstacles;
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
