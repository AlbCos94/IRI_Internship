//propies
#include "Search_closest_point.h" 


using namespace std;


void Search_closest_point (float pos_X, float pos_Y, vector <float> path_X, vector <float> path_Y, float start_i, float& target_i, float& dist)
{
	//establim primer uns valors molt grans
	target_i=99999999;
	dist=99999999;
	float dif;
	
	for (int i= start_i; i<(float)(path_X.size()); i=i+1)
	{
		dif= sqrt(pow((path_X[i] - pos_X),2) + pow((path_Y[i] - pos_Y),2)); // modul vector uneix parell de punts
		
		if (dif<dist)
		{
			dist=dif;
			target_i=i;
		}
		
		else if (i > target_i + 10)
		{
				break; // si ja s'ha trobat el minim ,  no te sentit continuan iterant
		}
	}
}




void Search_closest_point_full (float pos_X, float pos_Y, vector <float> path_X, vector <float> path_Y, float start_i, float& target_i, float& dist)
{
	//establim primer uns valors molt grans
	target_i=99999999;
	dist=99999999;
	float dif;
	
	for (int i= start_i; i<(float)(path_X.size()); i=i+1)
	{
		dif= sqrt(pow((path_X[i] - pos_X),2) + pow((path_Y[i] - pos_Y),2)); // modul vector uneix parell de punts
		
		if (dif<dist)
		{
			dist=dif;
			target_i=i;
		}
	}
}

