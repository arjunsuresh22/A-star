/*=================================================================
*
* planner.c
*
*=================================================================*/

#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <utility>
#include<time.h>

using namespace std;

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ROBOT_IN	prhs[1]
#define	GOAL_IN     prhs[2]


/* Output Arguments */
#define	ACTION_OUT	plhs[0]

/*access to the map is shifted to account for 0-based indexing in the map, whereas
1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)*/
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

/* Primitives Information */
#define NUMOFDIRS 8
#define NUMOFPRIMS 5
#define NUMOFINTERSTATES 10
#define NUMOFDIM 3
#define M_PI 3.141529
#define RES 0.1

typedef float PrimArray[NUMOFDIRS][NUMOFPRIMS][NUMOFINTERSTATES][NUMOFDIM];

int temp = 0;



struct cell
{
	short int cellposx, cellposy, celldir, prim;
	short int f, g, h;
	float currentposeX, currentposeY, currentposeTheta;

};

struct Comp {
	bool operator()(const cell& rhs, const cell& lhs)
	{
		return rhs.f > lhs.f;
	}
};

struct Dcell
{
	short int discX, discY;
	float contX, contY;
	short int g;

};
struct CompD {
	bool operator()(const Dcell& rhs, const Dcell& lhs)
	{
		return rhs.g > lhs.g;
	}
};

bool applyaction(double *map, int x_size, int y_size, float robotposeX, float robotposeY, float robotposeTheta,
	float *newx, float *newy, float *newtheta, PrimArray mprim, int dir, int prim)
{
	
	int i;
	for (i = 0; i < NUMOFINTERSTATES; i++) {
		*newx = robotposeX + mprim[dir][prim][i][0];
		*newy = robotposeY + mprim[dir][prim][i][1];
		*newtheta = mprim[dir][prim][i][2];

		int gridposx = (int)(*newx / RES + 0.5);
		int gridposy = (int)(*newy / RES + 0.5);

		/* check validity */
		if (gridposx < 1 || gridposx > x_size || gridposy < 1 || gridposy > y_size) {
			return false;
		}
		if ((int)map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0) {
			return false;
		}
	}


	return true;
}


bool applymotion(double *map, int x_size, int y_size,short int X, short int Y, short int &gridposx, short int &gridposy, int primD)
{
	

	switch (primD)
	{
	case 0:
		gridposx = X;
		gridposy = Y + 1;
		break;
	case 1:
		gridposx = X;
		gridposy = Y - 1;
		break;
	case 2:
		gridposx = X + 1;
		gridposy = Y ;
		break;
	case 3:
		gridposx = X - 1;
		gridposy = Y;
		break;
	}

	if (gridposx < 1 || gridposx > x_size || gridposy  < 1 || gridposy  > y_size)
	{
		return false;
	}

	if ((int)map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0)
	{
		return false;
	}

	return true;
}



int getPrimitiveDirectionforRobotPose(float angle)
{
	/* returns the direction index with respect to the PrimArray */
	/* normalize bw 0 to 2pi */
	if (angle < 0.0) {
		angle += 2 * M_PI;
	}
	int dir = (int)(angle / (2 * M_PI / NUMOFDIRS) + 0.5);
	if (dir == 8) {
		dir = 0;
	}
	return dir;
}

float absoluteValue(float value)
{
	if (value >= 0)
	{
		return value;
	}
	else
	{
		return -value;
	}
}

getHeuristic(float goalposeX, float goalposeY, const int x_size, const int y_size, double* map, std::vector<int> &hValue)
{	
	bool ret;
	short int gridposx, gridposy;

	bool closedList[x_size][y_size];
	for (int i = 0; i < x_size; i++)
	{
		for (int j = 0; j < y_size; j++)
		{
			closedList[i][j] = false;
			
		}
	}
	

	short int X = (short int)(goalposeX / RES + 0.5);
	short int Y = (short int)(goalposeY / RES + 0.5);
	

	Dcell currentNode;
	currentNode.discX = X;
	currentNode.discY = Y;
	currentNode.g = 0;
	hValue[(Y - 1)*x_size + (X - 1)] = 0;

	

	std::priority_queue<Dcell, std::vector<Dcell>, CompD> openList;

	openList.push(currentNode);
	
	
	while (!openList.empty())
	{
		//Remove the cell that you're expanding from the queue
		////state to expand and also checking for doubles in openlist.
		currentNode = openList.top();
		if (closedList[currentNode.discX - 1][currentNode.discY - 1] == true)
		{
			openList.pop();
			continue;
			//currentNode = openList.top();
		}
		//remove the cell that you're expanding from the queue
		
		openList.pop();
		
		//Closed list updated here
		closedList[currentNode.discX - 1][currentNode.discY - 1] = true;
		
		for (int primD = 0; primD < 4; primD++)
		{
			ret = applymotion(map, x_size, y_size, currentNode.discX, currentNode.discY, gridposx, gridposy, primD);

			if (ret)
			{
				//mexPrintf("DJIKSTRA flag: obstacle check satisfied: %d \n", ret);
				
				if (closedList[gridposx-1][gridposy-1] == false)
				{ 
					
					if (hValue[x_size*(gridposy - 1) + gridposx - 1] > (hValue[x_size*(currentNode.discY - 1) + currentNode.discX - 1] + 1))
					{
						
						//Update value of g ,X,Y
						Dcell successorNode;
						successorNode.g = currentNode.g + 1;
						successorNode.discX = gridposx;
						successorNode.discY = gridposy;
						hValue[x_size*(gridposy - 1) + gridposx - 1] = hValue[x_size*(currentNode.discY - 1) + currentNode.discX - 1] + 1;
						
						//Add to open List
						openList.push(successorNode);
					}
				}
			}
		}

		
		
	}

	

}

static void planner(
	double*	map,
	 int x_size,
	 int y_size,
	float robotposeX,
	float robotposeY,
	float robotposeTheta,
	float goalposeX,
	float goalposeY,
	PrimArray mprim,
	int *prim_id)
{
	mexPrintf("temp=%d\n", temp);
	temp = temp + 1;


	//Defined Local vaiables
	float thresh = 0.5;
	short int dir;
	float newx, newy, newtheta;
	bool ret;


	//Initializing Closed List & Initialized the g-value of each cell to infiniti
	bool closedList[x_size][y_size];
	std::vector<int> hValue;
	for (int i = 0; i < x_size; i++)
	{
		for (int j = 0; j < y_size; j++)
		{
			hValue.push_back(20000);
		}
	}
	std::vector<int> gValue;
	for (int i = 0; i < x_size; i++)
	{
		for (int j = 0; j < y_size; j++)
		{
			closedList[i][j] = false;
			gValue.push_back(20000);
		
		}
	}
	
	
	//Initializing parameters of starting node AND successor Node
	
	short int X = (short int)(robotposeX / RES + 0.5); //Discrete positions lie from 1 to x_size
	short int Y = (short int)(robotposeY / RES + 0.5); //Discrete positions lie from 1 to y_size
	dir = getPrimitiveDirectionforRobotPose(robotposeTheta);
	
	cell successorNode;
	

	cell currentNode;
	currentNode.f = 0;
	currentNode.g = 0;
	currentNode.h = 0;
	currentNode.cellposx = X;
	currentNode.cellposy = Y;
	currentNode.celldir = dir;
	currentNode.currentposeX = robotposeX;
	currentNode.currentposeY = robotposeY;
	currentNode.currentposeTheta = robotposeTheta;
	currentNode.prim = 0;
	
	gValue[(Y-1)*x_size + (X-1)] = 0;
	
	//Lets define prioity queue for open list
	std::priority_queue<cell, std::vector<cell>, Comp> openList;

	//Lets put start cell in open list
	openList.push(currentNode);
	
	//First iteration condition for primeID
	bool first_iter = true;

	//Heuristic-Djikstra search
	getHeuristic( goalposeX, goalposeY, x_size, y_size, map, hValue);
	FILE *fp;
	fp = fopen("Output.txt", "w");
	for (int i=0;i<y_size;i++)
	{
	     for (int j=0;j<x_size;j++)
		 {
	         fprintf(fp,"%d,",hValue[x_size*i+j]);
	     }
	     fprintf(fp,"\n");
	}


	
	// Main Loop for A-star algorithm
	while (!openList.empty())
	{	
		currentNode = openList.top();
		if (closedList[currentNode.cellposx - 1][currentNode.cellposy - 1] == true)
		{
			openList.pop();
			continue;
			
		}
		//remove the cell that you're expanding from the queue
		openList.pop();
		
		//Closed list updated here
		closedList[currentNode.cellposx - 1][currentNode.cellposy - 1] = true;

		if (absoluteValue(currentNode.currentposeX - goalposeX) < thresh && absoluteValue(currentNode.currentposeY - goalposeY) < thresh)
		{
			break;
		}
		
		
		mexPrintf("flag : satisfied threshold condition \n");
		for (int prim = 0; prim < NUMOFPRIMS; prim++)
		{
			ret =  applyaction(map, x_size, y_size, currentNode.currentposeX, currentNode.currentposeY, currentNode.currentposeTheta, &newx, &newy, &newtheta, mprim, currentNode.celldir, prim);

			if (ret)
			{
				mexPrintf("flag: obstacle check satisfied: %d \n", ret);
				short int gridposx = (short int)(newx / RES + 0.5);
				short int gridposy = (short int)(newy / RES + 0.5);
				short int newdir =(short int)getPrimitiveDirectionforRobotPose(newtheta);
			
				  if (closedList[gridposx-1][gridposy-1] == false)
				  {
					
					if (gValue[x_size*(gridposy - 1) + gridposx-1] > currentNode.g+1)
					{
						//mexPrintf("Flag: Admissibility Satisfied \n");
					
						successorNode.g = currentNode.g + 1;
						successorNode.h = hValue[x_size*(gridposy - 1) + gridposx - 1] ;
						successorNode.f = successorNode.g + successorNode.h;
						successorNode.cellposx = gridposx;
						successorNode.cellposy = gridposy;
						successorNode.celldir = newdir;
						successorNode.currentposeX = newx;
						successorNode.currentposeY = newy;
						successorNode.currentposeTheta = newtheta;
						gValue[x_size*(gridposy - 1) + gridposx-1] = currentNode.g + 1;
						
						//Only updating primid when its first iter or come from a different primid root

						if (first_iter)
						{
							successorNode.prim = prim;
							
						}
						else
						{
							successorNode.prim = currentNode.prim;
						}
					


						//Add to Open list
						openList.push(successorNode);
					}
				  }
			}
		}
		first_iter = false;
		
		
		
	
	}
	
	//get primitive id
	*prim_id = currentNode.prim;
	
	mexPrintf("action=%d\n", *prim_id);
	return;

}
void parseMotionPrimitives(PrimArray mprim)
{
	FILE * fp;
	fp = fopen("unicycle_8angles.mprim", "r+");
	char skip_c[100];
	int skip_f;
	float resolution;
	int num_angles;
	int num_mprims;
	fscanf(fp, "%s %f", skip_c, &resolution);
	fscanf(fp, "%s %d", skip_c, &num_angles);
	fscanf(fp, "%s %d", skip_c, &num_mprims);

	int i, j, k;
	for (i = 0; i < NUMOFDIRS; ++i) {
		for (j = 0; j < NUMOFPRIMS; ++j) {
			fscanf(fp, "%s %d", skip_c, &skip_f);
			for (k = 0; k < NUMOFINTERSTATES; ++k) {
				fscanf(fp, "%f %f %f", &mprim[i][j][k][0], &mprim[i][j][k][1], &mprim[i][j][k][2]);
			}

		}
	}
}

void mexFunction(int nlhs, mxArray *plhs[],
	int nrhs, const mxArray*prhs[])

{

	/* Read motion primtives */
	PrimArray motion_primitives;
	parseMotionPrimitives(motion_primitives);

	/* Check for proper number of arguments */
	if (nrhs != 3) {
		mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs",
			"Three input arguments required.");
	}
	else if (nlhs != 1) {
		mexErrMsgIdAndTxt("MATLAB:planner:maxlhs",
			"One output argument required.");
	}

	/* get the dimensions of the map and the map matrix itself*/
	int x_size = mxGetM(MAP_IN);
	int y_size = mxGetN(MAP_IN);
	double* map = mxGetPr(MAP_IN);

	/* get the dimensions of the robotpose and the robotpose itself*/
	int robotpose_M = mxGetM(ROBOT_IN);
	int robotpose_N = mxGetN(ROBOT_IN);
	if (robotpose_M != 1 || robotpose_N != 3) {
		mexErrMsgIdAndTxt("MATLAB:planner:invalidrobotpose",
			"robotpose vector should be 1 by 3.");
	}
	double* robotposeV = mxGetPr(ROBOT_IN);
	float robotposeX = (float)robotposeV[0];
	float robotposeY = (float)robotposeV[1];
	float robotposeTheta = (float)robotposeV[2];

	/* get the dimensions of the goalpose and the goalpose itself*/
	int goalpose_M = mxGetM(GOAL_IN);
	int goalpose_N = mxGetN(GOAL_IN);
	if (goalpose_M != 1 || goalpose_N != 3) {
		mexErrMsgIdAndTxt("MATLAB:planner:invalidgoalpose",
			"goalpose vector should be 1 by 3.");
	}
	double* goalposeV = mxGetPr(GOAL_IN);
	float goalposeX = (float)goalposeV[0];
	float goalposeY = (float)goalposeV[1];

	/* Create a matrix for the return action */
	ACTION_OUT = mxCreateNumericMatrix(1, 1, mxINT8_CLASS, mxREAL);
	int* action_ptr = (int*)mxGetData(ACTION_OUT);

	/* Do the actual planning in a subroutine */
	planner(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY, motion_primitives, &action_ptr[0]);

	return;

}