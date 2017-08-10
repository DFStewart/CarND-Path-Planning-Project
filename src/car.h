#ifndef CAR_H
#define CAR_H

#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

class Car
{

public:
    double pos_s;
	double pos_d;
	double vel_s;
	double vel_d;
	double accel_s;
	double accel_d;

	double pos_x;
	double pos_y;
	double yaw;
	double spd;

	double dist_inc;
	double prox;

	int    lane;

private:

};

#endif
