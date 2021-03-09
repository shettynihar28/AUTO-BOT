#include <stdio.h>
#include <stdlib.h>


#include "Auto_Function.h"

WifibotClient robot;
SensorData sensors_data;

/*departure in 0,0 to test : increase of the value of + how much ?*/
#define lim_x 50000 //limit of the test room (end of corridor)
#define lim_y 10000 //displacemet limit in test
position pos;//variable for position 
int IR_MAX = 100;/*Max limit for IR value*/

double old_right = 0;
double old_left = 0;
double delta_wheel_right = 0;
double delta_wheel_left = 0;

int space = 30;//measurement between wheels
double cpt_x = 0;//counter in -x 
bool flag_correction = false;

/*legacy function*/


void Robot_Connect()
{
	bool rep = robot.ConnectToRobot(IP_ADRESSE, PORT);
	if (rep)
	{
		printf("Connection failed...\n");
		getchar();
		return;
	}
	else
	{
		printf("Connection established...\n");

		
	}
}

void inistruc(position *p) {/*initialization of the structure*/
	p->O = 0;
	p->x = 0;
	p->y = 0;
}

void Robot_Forward(UINT16 speed_left, UINT16 speed_right)
{
	unsigned char flags = 128 + 32 + 64 + 16 + 1;
	robot.SendCommand(speed_left, speed_right, flags);
}

void Robot_backward(UINT16 speed_left, UINT16 speed_right)
{
	unsigned char flags = 128 + 0 + 32 + 0 + 1;
	robot.SendCommand(speed_left, speed_right, flags);
}

void Robot_turn_left(UINT16 speed_left, UINT16 speed_right)
{
	unsigned char flags = 128 + 0 + 32 + 16 + 1;
	robot.SendCommand(speed_left, speed_right, flags);
}

void Robot_turn_right(UINT16 speed_left, UINT16 speed_right)
{
	unsigned char flags = 128 + 64 + 32 + 0 + 1;
	robot.SendCommand(speed_left, speed_right, flags);
}

void Robot_stop(UINT16 speed_left, UINT16 speed_right)
{
	unsigned char flags = 128 + 64 + 32 + 16 + 1;
	robot.SendCommand(speed_left, speed_right, flags);
}

bool Robot_obstacle_right()
{
	if (sensors_data.IRRight > IR_MAX)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Robot_obstacle_left()
{
	if (sensors_data.IRLeft > IR_MAX)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Detection_obstacle()
{
	if (sensors_data.IRLeft > IR_MAX || sensors_data.IRRight > IR_MAX) 
	{
		return true;
	}
	else 
	{
		return false;
	}
}


void Robot_Forward_ticks(UINT16 speed_left, UINT16 speed_right, int movement_tick)/*move forward for given number of ticks*/
{
	int val_tick_right = 0;
	int val_tick_left = 0;
	int val_tick_right_depart = 0;
	int val_tick_left_depart = 0;

	robot.GetSensorData(&sensors_data);//acquisition
	val_tick_right = val_tick_right_depart = sensors_data.OdometryRight;//actualization
	val_tick_left = val_tick_left_depart = sensors_data.OdometryLeft;//actualization

	while ((val_tick_right < movement_tick + val_tick_right_depart) && (val_tick_left < movement_tick + val_tick_left_depart)) 
	{

		Robot_Forward(speed_left, speed_right);
		//robot.GetSensorData(&sensors_data);//acquisition
		updatesensors();
		val_tick_right = sensors_data.OdometryRight;//actualization
		val_tick_left = sensors_data.OdometryLeft;//actualization
		Sleep(100);
	}
}

void Robot_Turn_left_tick(UINT16 speed_left, UINT16 speed_right, int movement_tick)
{
	int val_tick_right = 0;
	int val_tick_right_depart = 0;
	int val_tick_left = 0; //variable for tick measurment
	int val_tick_left_depart = 0;

	robot.GetSensorData(&sensors_data);//aquisition
	val_tick_left = val_tick_left_depart = sensors_data.OdometryLeft;
	val_tick_right = val_tick_right_depart = sensors_data.OdometryRight;


	while ((val_tick_right < movement_tick + val_tick_right_depart) && (val_tick_left < movement_tick + val_tick_left_depart))
	{

		Robot_turn_left(speed_left, speed_right);
		//robot.GetSensorData(&sensors_data);//acquisition
		updatesensors();
		val_tick_left = sensors_data.OdometryLeft;//actualization
		val_tick_right = sensors_data.OdometryRight;//actualization
		Sleep(100);
	}
}

void Robot_Turn_right_tick(UINT16 speed_left, UINT16 speed_right, int movement_tick)//normalement fonctionne en relatif
{
	int val_tick_left = 0; 
	int val_tick_left_depart = 0;

	int val_tick_right = 0;
	int val_tick_right_depart = 0;

	robot.GetSensorData(&sensors_data);
	val_tick_left = val_tick_left_depart = sensors_data.OdometryLeft;
	val_tick_right = val_tick_right_depart = sensors_data.OdometryRight;

	while ((val_tick_left < movement_tick + val_tick_left_depart) && (val_tick_right < movement_tick + val_tick_right_depart))
	{

		Robot_turn_right(speed_left, speed_right);
		//robot.GetSensorData(&sensors_data);
		updatesensors();
		val_tick_left = sensors_data.OdometryLeft;
		val_tick_right = sensors_data.OdometryRight;
		Sleep(100);
	}
	
}

void updatesensors(void) 
{
	robot.GetSensorData(&sensors_data);
	printf("Batterie : %d\n", sensors_data.BatVoltage);
	printf("IRLeft : %d\n", sensors_data.IRLeft);
	printf("IRRight : %d\n", sensors_data.IRRight);
	printf("odoLeft : %d\n", sensors_data.OdometryLeft);
	printf("odoRight : %d\n", sensors_data.OdometryRight);
	
	mesure_odometry();//when called, each time measures new coordinates
	printf("x : %f\n", getx(&pos));
	printf("y : %f\n", gety(&pos));
	printf("O: %f\n", getO(&pos));
	
	//Sleep(100);
	
	verif_limits_xy(&pos);//check if the robot has reached a limit according to updated coordinates
}

/* new robot position update (x, y, O)
 /* by approximation of arc of circle */
void calcul_position_arc(position *p, double distance, double angle)
{
	/*radius and angle of the arc */
	double r, a;
	/* coordinates of the center of the arc */
	double xo, yo;

	if (angle == 0)
	{
		p->x += distance * cos(p->O);
		p->y += distance * sin(p->O);
	}
	else
	{
		/* calculation of the characteristics of the arc */
		a = angle / space;
		r = distance / a;

		/* center of the arc */
		xo = p->x - r * sin(p->O);
		yo = p->y + r * cos(p->O);

		/* Robot Co-ordinates */
		p->O += a;//origination
		p->x = xo + r * sin(p->O);//x
		p->y = yo - r * cos(p->O);//y

		
	}
}

/* delta_right_wheel and delta_left_wheel are the distance
 * traveled by the right and left wheels in a cycle */
void odometry(position *p, double delta_wheel_right, double delta_wheel_left)
{
	double delta_distance = 0, delta_angle = 0;

	delta_distance = (double)((delta_wheel_right + delta_wheel_left)) / 2;
	delta_angle = (double) (delta_wheel_right - delta_wheel_left);

	
	calcul_position_arc(p, delta_distance, delta_angle);
}

void mesure_odometry(void) 
{
	/*measures the distance traveled by the two sets of wheels*/
			

	delta_wheel_right = sensors_data.OdometryRight - old_right;//calculate distance
	delta_wheel_left = sensors_data.OdometryLeft - old_left;



	if ((delta_wheel_right > 500) || (delta_wheel_left > 500)) 
	{/*fix to filter to the peak odometeric value i.e (<25000) from the delta distance to the robot start*/
		printf("bad\n");
	}
	else 
	{
		odometry(&pos, delta_wheel_right, delta_wheel_left);
	}

	old_right = sensors_data.OdometryRight;
	old_left = sensors_data.OdometryLeft;

	delta_wheel_right = 0;
	delta_wheel_left = 0;
}

/*initiate structure*/
double getx(position *p) 
{
	return p->x;
}
double gety(position *p) 
{
	return p->y;
}
double getO(position *p) 
{
	return p->O;
}

void verif_limits_xy(position *p) 
{/*verify that the robot does not reach the x ​​and y limits*/

	/*replace all doubled in length */
	/*fixing problem of -x distance*/
	if (getx(&pos) < 0)
	{//if x is in the -
		/*it takes the max value at which the robot left in the negative*/
		if (getx(&pos) < cpt_x) 
		{
			cpt_x = getx(&pos);//max in negative
		}
	}
	
	if ((abs((long)getx(&pos))+(abs((long)cpt_x))) > lim_x)
	{ // each passage is added as the possible derivative in -x(cptx) to the +x direction
		/*fix to stop the robot*/
		while (1) {}
	}
	
	
}

int DistanceObstacle(side_IRSens side, int *LUT)/*returns the distance of the obstacle with reference to look up table*/
{
	if (side == LEFT)
	{
		return LUT[sensors_data.IRLeft];
	}
	else
	{
		return LUT[sensors_data.IRRight];
	}
}

void Displacement(int right, int left) 
{/**/
	unsigned char flags = 128 + 32 + ((left >= 0) ? 1 : 0) * 64 + ((right >= 0) ? 1 : 0) * 16 + 1;
	int right_1, left_1;

	right_1 = abs((int)(255 * right/ 100.f));
	left_1 = abs((int)(255 * left / 100.f));
	robot.SendCommand(right_1, left_1, flags);
	
	updatesensors();
	Sleep(100);

}