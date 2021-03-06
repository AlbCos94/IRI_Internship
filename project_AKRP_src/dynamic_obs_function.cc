//propies
#include "dynamic_obs_function.h" 

using namespace std;

float dynamic_obs_function (float dist)
{
	float lambda=0.5; //PARAMETRE
	float f=1; // (lo unic que varia del static_obs_function)
	float threshold=0.01; //PARAMETRE si el vehicle es troba a 1 cm del obstaculo, el cost sera altament penalitzat 
	float cost=0; //cost que tindra associat
	float penalization=1000;  //penalitzacio que s'aplicara si es supera la distancia de threshold
	
	if (dist<threshold) // si es troba el cotxe molt aprop del obstacle es penalitzara el cost asociat
	{
		cost=(f*exp((-1/lambda)*dist))+penalization;
		return cost;
	}
	else
	{
		cost=f*exp((-1/lambda)*dist);
		return cost; 
	}
}

