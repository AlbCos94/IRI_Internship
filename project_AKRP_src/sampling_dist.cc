#include "sampling_dist.h" 

using namespace std;

float sampling_dist (float vel, float  max_braking_accel, float security_distance) 
{
	return ceil( ((0.5*pow(vel,2))/max_braking_accel) + security_distance);

}

