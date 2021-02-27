#ifndef __C_OBSTACLE_H_INCLUDED__
#define __C_OBSTACLE_H_INCLUDED__

#include <math.h>
#include <iostream>
#include <algorithm>

using namespace std; 

//Creacio classe pels obstacles C_Obstacle
	
class C_Obstacle //creacio de la classe obstacle, els objectes d'aquesta classe contindran els obstacles )'cercles') que tingui la carretera (definits per coordenades del centre, radi i identificador)
{
	public: 
		// definim els membre de la clase (tots seran d'acces public)
		float pos[2]; // array de dos dimensions que contindra les coordenades (X i Y) de l'objecte (circunferencia)
		float r; // radi de la circunferencia que representa l'obstacle
		int id; // identificador per a identificar l'obstacle
		
		//constructor
		C_Obstacle(float pos1[2], float r1, int id1);
};

#endif // !__C_OBSTACLE_H_INCLUDED__
