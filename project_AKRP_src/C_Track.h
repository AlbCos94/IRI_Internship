#ifndef __C_TRACK_H_INCLUDED__
#define __C_TRACK_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>  
//propies
#include "C_Traj.h" 

using namespace std; 

//Creacio classe trajectoria C_Track
	
class C_Track //creacio de la classe carretera, els objectes d'aquesta classe contindran la carretera per on es circulara, amb els seus carrils
{
	public: 
		// definim els membre de la clase (tots seran d'acces public)
		float lane_width; // amplada del carril (metres)
		int  num_lanes; // numero de carrils que hi han a la carretera
		
		vector <C_Traj > center_paths; //vector de elements de la classe C_Traj (cada un representa un carril (X's Y's i k's del carril)
		vector <float> num_points; //vector que conte la longitud (numero de punts pels que esta formada la trajecteria de cada carril) --> nombre de indexos de la carretera 
		
		//constructor
		C_Track(float lane_width1, vector <C_Traj > center_paths1); // per inicialitzar aquests objectes, necessitara l'amplitud dels carrils i el conjunt de carrils representats per la classe C_Traj que conformen la carretera
};

#endif // !__C_TRACK_H_INCLUDED__
