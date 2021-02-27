#ifndef __MANDATORY_POINT_H_INCLUDED__
#define __MANDATORY_POINT_H_INCLUDED__

#include <math.h>
#include <iostream>
#include <vector>

#include "C_SamplePoint.h" 

using namespace std; 

// Representa un punt critic del circuit on haurem de fer un certa maniobra ( es troba a alla un obstacle) ?¿?
//Creacio estructura 
struct mandatory_point 
{
	// definim els membre de la estructura
	C_SamplePoint point; // element de la classe C_SamplePoint
	
	vector <float > traj; 
	float GP_path_station; // 'numero del punt' del carril en que ens trobem ?¿?¿?
	float reason; // 1--> changing lane, 2 --> static obstacle avoidance, 3 --> static obstacle avoidance
};

#endif // !__MANDATORY_POINT_H_INCLUDED__
