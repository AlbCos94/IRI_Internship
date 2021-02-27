#include "car_circles.h" 

using namespace std;

void car_circles (float car_l, float  car_w, float  car_r_ovh, float  pos_X, float  pos_Y, float  heading, int n, float& r, vector <vector<float> >& centers)
{
	//n = 3; Number of desired circles
	r = sqrt((car_l/pow(n,2.0)) + (pow(car_w,2.0)/4.0)); // calcul dels radis dels cercles
	
	float d = 2*sqrt(pow(r,2.0) - (pow(car_w,2.0)/4.0));
	float d_ovh = (car_l - (n - 1)*d)/2.0;
   
	centers[0][0] = pos_X + (-car_r_ovh + d_ovh)*cos(heading*M_PI/180); // x --> calcul de la coordenada x del centre del primer radi
	centers[1][0] = pos_Y + (-car_r_ovh + d_ovh)*sin(heading*M_PI/180); // y --> calcul de la coordenada y del centre del primer radi
	
	// Calcul de la resta de coordenades x i y dels centres del cercles
	for (int i =1;i<n;i++)
	{
		centers[0][i] = centers[0][i-1] + (d*cos(heading*M_PI/180));
		centers[1][i] = centers[1][i-1] + (d*sin(heading*M_PI/180));
	}

}
