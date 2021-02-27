#ifndef __C_LANECHANGE_H_INCLUDED__
#define __C_LANECHANGE_H_INCLUDED__

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
#include "G2_Spline.h" 
#include "C_Traj.h" 
#include "C_DynObstacle.h" 
#include "spline_vel.h" 
#include "dynamic_obs_function.h" 
#include "C_TrajGen.h" 
#include "C_Sampling.h" 

using namespace std; 

//Creacio classe per .....
class C_LaneChange
{
	public: 
		// membres de la clase 
		C_Sampling other_sampling; // % C_Sampling object of the other lane
		C_TrajGen other_traj_gen; // % C_TrajGen object of the other lane
		
		int other_lane_blocked; // % If the sampling in the other lane is blocked --> 1
		
		C_Traj other_best_traj; // % Best C_Traj of the other lane
		float other_best_traj_min_cost; // % its cost
		float other_best_traj_col; // % its collision
		
		vector <vector<float> > our_center_lane; // % Our center lane --> coordenades X's, Ys i curavatures
		vector <vector<float> > other_center_lane; // % Center of the other lane --> coordenades X's, Ys i curavatures

		vector<C_Obstacle> obstacles; // ¿?¿?¿?¿ C_StaticObs?¿?¿?¿? % All the obstacles from us to the ending point

		float  r_col; // % Collision radius
		float min_turn_r; // % Minimum turning radius
		C_SamplePoint initial_point; // % C_Sample_Point with the initial point
		float resolution; // % Resolution of the G-2 splines
		float sampling_distance; // % Interpolation distance
		
		//constructor 
		C_LaneChange ( vector <vector<float> > our_center_lane1, vector <vector<float> > other_center_lane1, float resolution1, float sampling_distance1, vector<float> dists1, vector<float> offsets1, C_SamplePoint initial_point1, float target_i_GP1, C_StaticObs static_obs1, float r_col1, float min_turn_r1);
		

		//funcions membre
		
		//retorna el valor de la variable 'other_lane_blocked' --> si es 1, vol dir que esta bloquejada i que per tant no es pot canviar a l'altre carril
		int lane_change_is_blocked();
		
		
		//generador de trajectoria per canviar de carril --> trajectoria que realitza el trasllat a un altre carril que no es el predefinit
		void gen_lane_change_trajs (float n, C_Traj our_best_traj, float max_pos[2], float des_sampling_distance, vector<C_Traj> &trajs, vector<float> &c_lengths, vector<float> &c_maxks, vector<float> &c_maxkdots, vector<float> &c_static_obs, vector<float> &collisions, int &lane_change_possible, C_SamplePoint &final_point);
				// input
					// n--> Number of trajectories created (they are equispaced)
					// our_best_traj --> C_Traj of our lane --> trajectoria que volem seguir
					// max_pos[2] --> [x, y] absolute position to finish the manoeuvres --> idicarem amb [-1,-1] per quan estigui buida
				// output
					// trajs--> vector d'elements de la clase 'C_Traj', cada element defineix una trajectoria candidata generada per a canviar al altre carril
					// c_lengths --> costos asociats a longitud trajectories
					// c_maxks --> ''
					// c_maxkdots --> ''
					// c_static_obs --> ''
					// NO ES TE EN COMPTE OFFSET COSTS !!
					// collisions --> vector on es produeixen les colisions (-
					// lane_change_possible --> indica amb un 1 si es pot fer canvi de carril
					// final_point --> element calse C_SamplePoint, amb les caracteristiques del ultim punt de la trajectoria (la cual acaba a l'altre carril)
		
		
		// escollir la millor trajetoria de les candidates generades per a realitzar el canvi de carril
		void best_lane_change_traj (vector<C_Traj> trajs, vector<float> c_lengths, vector<float> c_maxks, vector<float> c_maxkdots, vector<float> c_static_obs, vector<float> collisions, float comfort_a_lat, float max_vel_restriction[2], float vel, float max_road_vel, float a_0, float a_ant, float max_decel, float max_accel, float current_t, float delta_t, vector<C_DynObstacle> dyn_obs, C_Traj &best_traj, float &best_traj_cost, float &best_traj_col, vector<vector<float> > &best_traj_vp, float &best_traj_vp_cost, float &best_vp_accel, float &best_vp_finalvel);
				// input
					// max_vel_restriction [2]--> array que indica la maxima velocitat a assolir [m/s] (2on element del array) i a partir de quina distancia longitudinal s'ha d'assolir (metres) (1er element del array)

				// output
					// best_traj_vp --> millor perfil de velocitat --> vector format per 4 vectors 
																	// Punts --> vector amb les diferent posicion a cada instant
																	// Vels --> vector amb les diferent velocitats a cada instant
																	// Accels --> vector amb les diferents acceleracions a cada instant
																	// temps --> vector amb els instants de temps
		//
		void gen_avoidance ( float n, C_Traj our_best_traj, C_Traj best_traj_X, float max_pos[2], vector<C_Traj> &trajs, vector<float> &c_lengths, vector<float> &c_maxks, vector<float> &c_maxkdots, vector<float> &c_static_obs, vector<float> &collisions, int &lane_change_possible);  
				// input
					// float max_pos[2] --> val (-1, -1) si no en tenim 
		
		
		
		void gen_dyn_avoidance ( float n, C_Traj our_best_traj, vector<float> dists, vector<float> offsets, C_StaticObs static_obs, vector<C_Traj> &trajs, vector<float> &c_lengths, vector<float> &c_maxks, vector<float> &c_maxkdots, vector<float> &c_static_obs, vector<float> &collisions, int &lane_change_possible);  

		

		
		
		
};
#endif // !__C_LANECHANGE_H_INCLUDED__
