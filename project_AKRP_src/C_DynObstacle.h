#ifndef __C_DYNOBSTACLE_H_INCLUDED__
#define __C_DYNOBSTACLE_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>  

//propies
#include "C_Obstacle.h" 
#include "Search_closest_point.h" 

using namespace std; 

//Creacio classe per representar els obstacles móvils (altres vehícles en moviment)
class C_DynObstacle
{
	public: 
		// definim els membre de la clase (tots seran d'acces public)
		float t_0; // temps inicial a partir del qual es començara a moure
		float T; // temps total durant el qual es moura el vehicle (ve determinat per la distancia que recorrera, la qual es defineix en el constructor de la classe)
		float delta_t; // temps de mostratge
		float v; // velocitat que porta el vehicle / es suposara com a constant
		float r; // radi (radi de la circunferencia que representa el vehicle ?¿?¿)
		
		vector <vector<float> > path; // cami que seguira el vehícle (format per un vector de les coordenades X i un altre amb les coordenades Y)
		
		vector <vector<float> > positions; // vector format per 4 vectores (estructura algo diferent al matlab!)
											//0	// 1r vector --> coordenades x carril a seguir
											//1	// 2n vector --> coordenades y carril a seguir
											//2	// 3r vector --> instant de temps en que el vehicle es trobara en aquella coordenada (x,y) i el 'index' del carril corresponent 
											//3	// 4r vector --> indexs del carril a seguir ('numeros de punt')
										   
		// 'numero de punt del carril' --> equival a 'index del carril'
		
		vector <float> heading; // vector de orientacions del vehicle --> heading in deg of each point of the path, it is used for plotting a vehicle(NECESSARI ¿?¿?¿?)
		int id; // numero identificador del vehicle
		int visible; // per defecte val 0 --> val 1 mentre el vehicle s'esta movent
		int lane; // 0 if it is not in any lane, otherwise it is the lane number
		
		//constructor 
		C_DynObstacle(float t_01, float delta_t1, float v1, float r1, vector <vector<float> > path1, float dist, float res, int id1, int lane1); 
				
		//funcions membre
		void update_visibility(float current_t); // actualitza el membre 'visible', si introduim un instant de temps en que el cotze s'esta movent, farem que 'visible' valgui 1, de lo contrari farem que valgui 0
		
};

#endif // !__C_DYNOBSTACLE_H_INCLUDED__
