#ifndef __S_LANE_SAMPLING_COMB_H_INCLUDED__
#define __S_LANE_SAMPLING_COMB_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>  

//propies
#include "C_Obstacle.h" 
#include "Search_closest_point.h" 
#include "C_SamplePoint.h" 
#include "C_StaticObs.h" 
#include "C_Traj.h" 



using namespace std; 

//Creacio estructura per representar la discretitzacio espaial d'un carril
struct S_lane_sampling_comb
{
		// membres 
		vector <vector<C_SamplePoint> > traj_points; // matriu amb la discretitzacio espaial i la informacio de cadascun dels punts en que ha estat feta la discretització
		vector <vector<int> > feasible_traj_points; // matriu de 1s i/o 0s --> 1 indica que no hi ha cap obstacle en aquells punt, 0 es que hi ha obstacle
		vector<float> traj_stations; // numeros de punts (indexos del carril) on es troba cada una de les 'stations' amb que discretitzem el carril ( cada columna de 'traj_points' compartira la mateixa 'station' )
		vector<int> obstruction_here; // indica amb 0 la 'station' on no hi ha cap obstacle, i amb un 1 on hi ha obstacle (encara que sigui nomes en un dels seus punts de mostreig(offset))
		float max_feasible_station; // valor del index de la 'station' mes allunyada
		int lane_blocked; // indica amb un 1 que no es pot passar pel carril (totalment bloquejat) i amb un 0 de que si es pot pasar ( tot i que por tenir obstacles)
		float first_sampling_index; // index del vector 'traj_station' on acaba el primer mostreig fi
		vector<float> offset_combinations; // un 1 indica que podem realitzar totes les combinacions possibles de 'station' i un 0 que em de seguir la anterior
};
#endif // !__S_LANE_SAMPLING_COMB_H_INCLUDED__
