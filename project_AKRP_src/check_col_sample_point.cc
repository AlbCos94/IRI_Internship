#include "check_col_sample_point.h" 

using namespace std;

int check_col_sample_point (C_SamplePoint sample_point, float r, C_Obstacle obs)
{
	int col=0;
	float dist_centers; // distancia entre centres del cotxe i del obstacle
	dist_centers = sqrt( pow((obs.pos[0]-sample_point.x),2) + pow((obs.pos[1]-sample_point.y),2) );
	float true_dist; // distancia entre la periferia de les dues circunferencies
	true_dist=dist_centers-obs.r-r;
	
	if (true_dist <= 0) // els dos obstacle rocen o una circunferencia interseca o essta a dins de l'altre 
	{
		col=1;
	}
	
	return col;
}
