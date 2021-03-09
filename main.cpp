#include <stdio.h>
#include <stdlib.h>



#include "Auto_Function.h"
#include "WifibotClient.h"

bool flag = false; //TO know if an obstacle has been avoided
//long x_test = 0;

int SharpLUT[] = { 150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,103,102,101,100,99,
		  96,95,90,88,87,86,85,84,83,82,81,80,79,77,75,74,73,72,71,70,69,68,67,66,65,65,64,64,63,62,61,61,60,60,59,59,58,58,57,57,56,56,56,55,55,55,54,54,54,53,
		  53,53,52,52,51,51,50,50,49,49,49,48,48,47,47,46,46,46,45,45,44,44,43,43,42,42,41,41,41,40,40,40,39,39,39,39,39,38,38,
				  38,38,37,37,37,37,36,36,36,36,35,35,35,35,34,34,34,34,34,34,33,33,33,33,32,32,32,32,32,31,31,31,31,30,30,30,30,30,30,30,
		  30,29,29,29,29,29,29,29,28,28,28,28,28,28,27,27,27,27,27,27,27,26,26,26,26,25,25,25,25,25,25,24,24,24,24,24,23,23,23,
		  23,23,23,22,22,22,22,22,21,21,21,21,21,21,20,20,20,20,19,19,19,19,19,19,18,18,18,18,18,18,17,17,17,16,16,15,15 };


void main(void)
{
	/*.........................*/
	/* Connection to the robot */
	/*.........................*/

	Robot_Connect();//connection to the robot
	inistruc(&pos);//nitialization of the xyO structure to 0
	updatesensors();//updated sensors + coordinates
	
	/*..............*/
	/* Main program */
	/*..............*/

	while(1)
	{
		updatesensors();//updated sensors + coordinates
		if ((DistanceObstacle(LEFT, SharpLUT) <= 90) && (DistanceObstacle(RIGHT, SharpLUT) <= 90))
		{
			Displacement(-30, -30);
			//printf("stop");
			
			Displacement(-30, 30);
		}
		else if ((DistanceObstacle(LEFT, SharpLUT) <= 90) && (DistanceObstacle(RIGHT, SharpLUT) >= 90))
		{
			Displacement(-30, 30);
			flag = true;//i.e. detected an osbtacle
			
			printf("left");
		}
		else if ((DistanceObstacle(LEFT, SharpLUT) >= 90) && (DistanceObstacle(RIGHT, SharpLUT) <= 90))
		{
			Displacement(30, -30);
			flag = true;//i.e. detected an osbtacle
			
			printf("right");
		}
		else if ((DistanceObstacle(LEFT, SharpLUT) > 90) || (DistanceObstacle(RIGHT, SharpLUT) > 90))
		{
			Displacement(30, 30);
			printf("forward");
			//once the sensor detects nothing robot advances
			if (flag == true) {//i.e. detected an osbtacle

				
				Robot_Forward_ticks(50, 50, 3000);//bypass the obstacle for 3000 ticks(delay)
				while ((getO(&pos) <= -4.0) || (getO(&pos) >=0)) { //at the start the orientation it is fixed at -0.75
					/*if the robot turns its orientation will be changed */
					/*the robot will turn on itself as long as it does not fall back into an orientation range of 0.75 */

					if (getO(&pos) < -0.75) {
						Displacement(15, -15);//either to left
					}
					else {
						Displacement(-15, 15);//or to right
					}
				
				}
				Robot_stop(0,0);
				flag = false;//After overtaking the obstacle, robot resumes its way
			}
		}

		Sleep(100);

	}	
}



