#include "WifibotClient.h"


#define IP_ADRESSE "192.168.1.72"
#define PORT	15020


#pragma once



typedef struct
{
	double x = 0;    
	double y = 0;
	double O = 0;     
} position;

void Robot_Connect();
void Robot_Forward(UINT16 speed_left, UINT16 speed_right);
void Robot_backward(UINT16 speed_left, UINT16 speed_right);
void Robot_turn_left(UINT16 speed_left, UINT16 speed_right);
void Robot_turn_right(UINT16 speed_left, UINT16 speed_right);
void Robot_stop(UINT16 speed_left, UINT16 speed_right);
bool Robot_obstacle_right();
bool Robot_obstacle_left();
bool Detection_obstacle();

void Robot_Forward_ticks(UINT16 speed_left, UINT16 speed_right, int movement_tick);
void Robot_Turn_left_tick(UINT16 speed_left, UINT16 speed_right, int movement_tick);
void Robot_Turn_right_tick(UINT16 speed_left, UINT16 speed_right, int movement_tick);

void updatesensors(void);


void calcul_position_arc(position *p, double distance, double angle);
void odometry(position *p, double delta_way_right, double delta_way_left);
void mesure_odometry(void);

double getx(position *p);
double gety(position *p);
double getO(position *p);

void verif_limits_xy(position *p);
void inistruc(position *p);

extern position pos;//variable for position

extern UINT16 speed_left;
extern UINT16 speed_right;


extern enum side_IRSens { RIGHT, LEFT };
void Displacement(int right, int left);
int DistanceObstacle(side_IRSens side, int *LUT);

