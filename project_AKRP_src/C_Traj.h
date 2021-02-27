#ifndef __C_TRAJ_H_INCLUDED__
#define __C_TRAJ_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>  

// propies
#include "Search_closest_point.h"
#include "C_Obstacle.h" 
#include "static_obs_function.h" 
#include "funcions_mat.h" 

using namespace std; 

//Creacio classe trajectoria C_Traj
	
class C_Traj //creacio de la classe trajectoria, els objectes d'aquesta classe contindran les trajectories candidates (amb les seves caracteristiques)
{
	public: 
		// definim els membre de la clase (tots seran d'acces public)
		float resolution; // resolucio de la trajectoria
		
		
		float ini_point_sampling_coords[2]; // coordenades (columna i fila) --> (station i offset) (al reves del matlab) dels INDEXOS respecte la matriu 'traj_points' del punt inicial de la trajectoria 
											// per inicialitzar 'ini_point_sampling_coords', ho indiquem amb -1 i -1 (ens trobem fora de la matriu  de mostreig)
		float end_point_sampling_coords[2]; // coordenades (columna i fila) --> (station i offset) (al reves del matlab) dels INDEXOS respecte la matriu 'traj_points' del punt final de la trajectoria 
											
											
		vector <vector<float> > traj; // matrius (vector de vectors) --> Vector format per tres vectors, un contindra coodenades X, un altre Y i un altre curvatures
						
		float len; // longitud de la trajectoria
		float maxk; // maxima curvatura
		float maxkdot; // maxima derivada de la curvatura
		
		float cost_s; // cost estatic de la trajectoria
		float cost_d; // cost dinamic de la trajectoria
		
		//constructor
		C_Traj(float ini_point_sampling_coords1[2], float end_point_sampling_coords1[2], vector <vector<float> > traj1, float dist1, float res1);
		
		
		
		//default constructor
		C_Traj(); // inicialitza tot a 0, menys la 's' que la posa a -1
				
		//funcions membre
		void compute_cost_static_obs(vector <C_Obstacle> obstacle, float r_col, float &cost, float &col_index); //calcula es cost estatic total asociat de la trajectoria respecte els diferent obstacles que hi han
						//input
						// 'obstacle' es un vector d'element de la classe C_Obstacle
						// 'r_col' es el radi de colissio, vindra donat pel radi dels cercles que representen el cotxe host
						//output
						// 'cost' sera el cost estatic asociat a tots els obstacles estatics
						// 'col_index' sera el 'número de punt' (station) on es produeix colisio amb obstacle o on es troba el cotxe molt aprop d'un obstacle / si no hi ha, valdrà -1
		
		float get_static_cost(void); // retorna el cost estatic asociat als obtacles d'aquella trajectoria
};

#endif // !__C_TRAJ_H_INCLUDED__
