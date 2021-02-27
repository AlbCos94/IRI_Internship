#ifndef __C_MANDATORY_POINT_H_INCLUDED__
#define __C_MANDATORY_POINT_H_INCLUDED__

#include <math.h>
#include <iostream>
#include <vector>

#include "C_SamplePoint.h" 
#include "C_Traj.h" 

using namespace std; 

//Creacio clase
class C_mandatory_point 
{
	public:
	// definim els membre de la estructura
	C_SamplePoint point; // element de la classe C_SamplePoint
	C_Traj traj; // element que representa la trajectoria a seguir
	float GP_path_station; 
	float reason; // 0--> No estem a cap maniobra, 1--> maniobra de changing lane, 2 --> maniobra de static obstacle avoidance, 3 --> maniobra static obstacle avoidance, 5 --> Dynamic obstacle avoidance
	
	C_mandatory_point() //constructors de la classe del membre 'point' (membre de la classe C_SamplePoint)
						: point(0, 0, 0, 0, -1) {
	}
	
};

#endif // !__MANDATORY_POINT_H_INCLUDED__
