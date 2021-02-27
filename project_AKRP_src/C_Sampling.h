#ifndef __C_SAMPLING_H_INCLUDED__
#define __C_SAMPLING_H_INCLUDED__

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
#include "S_lane_sampling.h" 
#include "S_lane_sampling_comb.h" 
#include "funcions_mat.h" 
#include "check_col_sample_point.h" 

using namespace std; 

//Creacio classe per .....
class C_Sampling
{
	public: 
		// membres de la clase 
		vector <vector<float> > center_lane; // vector amb les coordenades x, y i k
		float resolution; // Splines resolution
		float final_station; // maxima 'station' longitud total del tram de carretera a tractar
		vector <float > stations; // 'numeros de punt' (indexs) que determinen les 'stations' (longitudinal indices) of the center lane amb les quals discretitzem el tram [-]
		vector <float > offsets; // lateral offsets (meters)
		
		C_SamplePoint initial_point; // caracteritza l'estat X del cotxe
								// x; en metres
								// y; en metres
								// k; en 1/metres
								// head; degrees
								// s=-1  (index point)
								
		float initial_point_GP_path_i; // the index of the GP_path of the initial_point
		
		C_StaticObs static_obs;
		
													
		//constructor 
		C_Sampling( vector <vector<float> > center_lane1, float resolution1, float sampling_dist1, vector <float > dists1, vector <float > offsets1, C_SamplePoint initial_point1, float initial_point_GP_path_i1, C_StaticObs static_obs1);
		
		//default constructor
		C_Sampling();
		
		//funcions membre
		
		void sample (float initial_point_GP_path_i, vector <float > desired_stations, float r_col, vector <vector<C_SamplePoint> > &traj_points, vector <vector<int> > &feasible_traj_points, float &max_feasible_station, int &lane_blocked); // funcio per mostrejar
					//inputs
						// r_col --> radi de comprovacio per les colisions (radi que determina el cotxe)
						// desired_stations --> 'stations' que tractarem
						// initial_point_GP_path_i --> index del punt inicial del carril
					//outputs
						// traj_points --> matriu d'elements de la clase 'C_SamplePoint' que representa tota la discretitzacio espaial del tram de carril
						// feasible_traj_points --> matriu de 1 i/o 0s, indica en cada punt de mostreig si hi ha a alla obstacle (amb un 0)
						// max_feasible_station --> index de la estation mes allunyada del tram
						// lane_blocked --> indica si el carril esta totalment bloquejat (amb un 1)
		
		
		
		// afegim stations a les stations propies del mostreig espaial inicial, fins a la distancia de captacio total (lookahead distance) --> segons siobstacles estatics i un station a la distancia final de captacio del seonsor
			//Un cop hem emprat el vector 'dists' per fer una primera discretitzacio del carril (la del tram mes proper) 
			// faig una altre discretitzaci fins al punt final determinat per la 'lookahead_dist' --> abast del nostre sensor
			// aquesta segona discretitzacio es mes basta --> s'incloura una 'station' al final i unes altres entre la 1a discretitzacio i la final
			// aquestes altres sempre seran augmentant la ultima el valor de la 'station'
		void sample_ahead (float initial_point_GP_path_i, float desired_station, float r_col, vector <vector<C_SamplePoint> > &traj_points, vector <vector<int> > &feasible_traj_points, float &max_feasible_station, int &lane_blocked);
		
		// % Function to sample ahead, only samples 1 time -
		void sample_ahead_other_lane (float initial_point_GP_path_i, float desired_station, float r_col, vector <vector<float> > other_center_lane, vector <vector<C_SamplePoint> > &traj_points, vector <vector<int> > &feasible_traj_points, float &max_feasible_station, int &lane_blocked);

		// funcio per a mostrejar un tram de carril
		void sample_lane (float r_col, S_lane_sampling& lane_sampling); 

		//% Function to compute the combinations from the sampling points --> Construim vector que indica quines 'stations' han de seguir el mateix offset de la 'station' previa degut a la presencia de obstacles --> 'offset_combinations'
		void traj_combinations(S_lane_sampling lane_sampling, S_lane_sampling_comb& new_lane_sampling );
					// input --> lane_sampling --> tram de carril mostrejat , element del tipus S_lane_sampling
					// output --> element del tipus 'S_lane_sampling_comb' --> igual que 'S_lane_sampling', pero inclou vector que conte 1's per indicar que la trajectoria generada ha de tenir el mateix offset que les trajectories generades de la 'station' anterior
				
				
		// Evitar una carril?¿?					
		void avoidance_other_lane (float r_col, vector <vector<float> > other_center_lane, float collision_index, S_lane_sampling &lane_sampling, int &lane_change_finished); 

		
		
		
		
					
		
};
#endif // !__C_SAMPLING_H_INCLUDED__
