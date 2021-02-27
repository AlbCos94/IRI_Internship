#ifndef __C_TRAJGEN_H_INCLUDED__
#define __C_TRAJGEN_H_INCLUDED__

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

using namespace std; 

//Creacio classe per .....
class C_TrajGen
{
	public: 
		// membres de la clase 
		float resolution; // % Splines resolution, in [m] --> metres que hi han entre dos punts
		
		C_SamplePoint initial_point; //Punt inicial
		float initial_point_GP_path_i; // % The index of the GP_path of the initial_point
		C_StaticObs static_obs;
		
		vector <vector<C_SamplePoint> > traj_points; // % 2D matrix with the points ( type C_SamplePoint )
		vector <vector<int> > feasible_traj_points; // % 2D matrix mask (1's punts no pertorbats per obstacles)
		vector<float> traj_stations; // % Vector with the stations ( te el index de punt de cada 'station')
		vector<int> obstruction_here; // % Vector indicating where are the true obstructions due to an static obstacle
		vector<float> offset_combinations; // % Vector indicating where are the combinations with the different offsets --> 0 per quan s'ha de mantenir el offset escollit a la 'station' anterior, si no s'ha de mantenir el offset vadra 1 per a aquella station
		
		float first_sampling_index;  // % Station index of the first sampling --> index del vector 'traj_stations', que indica en quina 'station' acaba la discretitzacio 'fina' realitzada --> la provocada pel vector 'dists'
		
		int num_stations;           // % Number of stations
        int num_offsets;            // % Number of offsets
        float max_feasible_station; // % Maximum feasible station
        int lane_blocked;           // % If the lane is blocked --> 1
		
													
		//constructor 
		C_TrajGen( S_lane_sampling_comb lane_sampling1, C_SamplePoint initial_point1, float initial_point_GP_path_i1, float resolution1, C_StaticObs static_obs1);
		
		//default constructor
		C_TrajGen();
		
		
		//funcions membre
		
		// genera la optima combinacio de trajectories possibles FINS LA 1a STATION
		void generate_initial_opt (float r_col, float min_turn_r, vector<C_Traj> &initial_trajs); 
					//inputs
						// r_col --> radi de comprovacio per les colisions (radi que determina el cotxe)
						// min_turn_r --> minim radi de gir (metres) (PARAMETRE DEL COTXE)
					//outputs
						// initial_trajs --> vector d'elements de la clase 'C_Traj', cada element defineix la trajectoria fins a cada un dels offset de la 'station' proxima 
			
			// Si no hi ha cap obstacle a la 'station' inicial. retorna un vector de trajectories (elements classe C_Traj) als offsets de la primera 'station'
			// Si hi ha obstacle a al primera 'station', generem el conjunt de trajectories a les diverses 'stations' i agafem la de 'cost minim'
		
		// genera la resta de trajectories de les 'stations' del espai mostrejat ampliat --> 'stations' que no pertanyen al espai discretitzat fi
		void generate_rest (vector<C_Traj> &trajs); 
				//output
					// vector de traejctories finas al final del mostreig ampliat (fins a la lookahead_distance)
				
				
		//Generates the complete trajectories up to the final station, and also computes its costs 
		// Uneix les trajectories generades en la discretitzacio fina amb les trajectories generades amb la discretitzacio ampliada
		void merge (vector<C_Traj> initial_trajs, vector<C_Traj> rest_trajs, float r_col, vector<C_Traj> &trajs, vector<float> &c_lengths, vector<float> &c_maxks, vector<float> &c_maxkdots, vector<float> &c_offsets, vector<float> &c_static_obs, vector<float> &collisions);
					//inputs
						// initial_trajs --> trajectories generades fins a la 1a (station)
						// rest_trajs --> resta de trajectories generades fins a la final station ( nomes quan hi ha obstacle en el carril ) ?¿?¿
						// r_col --> radi de coprovacio de les colision (representa en el cotxe)
					//outputs
						// trajs --> vector d'elements de la clase 'C_Traj', cada element defineix la trajectoria fins a un dels end_point (fusiona les trajectories introduides
						// c_lengths --> vector de costos referits a les longituds de les trajectories de cada trajectoria
						// c_maxks --> vector de curvatures maximes de les trajectories generades
						// c_maxkdots --> vecrtor de derivades de curvatures maximes de les trajectories generades
						// c_offsets --> vector referenciats als costos provocats per quan ens desviem de la trajectoria centra del carril ?¿?¿
						// c_static_obs --> costos asociats als obstacles estatics asociats a cada trajectoria
						// collisions --> % For knowing if collision. -1 if not, otherwise it has the target_i ('station') of the collision
		
		//Escull la millor trajectoria (la que te associada un cost menor)
		void best_path (vector<C_Traj> trajs, vector<float> c_lengths, vector<float> c_maxks, vector<float> c_maxkdots, vector<float> c_offsets, vector<float> c_static_obs, vector<float> collisions, float min_turn_r, C_Traj &best_traj, float &min_cost, float &best_traj_col, float &traj_i);
					//inputs
						// trajs --> vector d'elements de la clase 'C_Traj', cada element defineix les trajectoreis candidates
						// c_lengths --> vector de costos referits a les longituds de les trajectories de cada trajectoria
						// c_maxks --> vector de curvatures maximes de les trajectories generades
						// c_maxkdots --> vecrtor de derivades de curvatures maximes de les trajectories generades
						// c_offsets --> vector referenciats als costos provocats per quan ens desviem de la trajectoria centra del carril ?¿?¿?
						// c_static_obs --> costos asociats als obstacles estatics asociats a cada trajectoria
						// collisions --> % For knowing if collision. -1 if not, otherwise it has the target_i ('station') of the collision
						// min_turn_r --> radi que representa el vehicle per chequejar obstacles
					//outputs
						// best_trajs --> trajectoria escollida
						// min_cost --> cost asociat a la trajecotria escollida ( la de cost minim )
						// best_traj_col -->  station on es produeix colisio de la trajectoria escollida ??¿
						// traj_i --> index del vector de trajectories que correspon a la trajectoria escollida
					
		void compute_vel_static_restrictions (C_Traj traj, float our_station, float collision, float min_turn_r, float comfort_a_lat, vector <vector<float> > &vel_restr);
					//inputs
						// trajs --> element de la classe 'C_Traj'
						// our_station --> station on ens trobem
						// collision --> indica si es produeix colisio, i en quina 'station' es produeix
						// min_turn_r --> minim radi de cruvatura cotxe
						// comfort_a_lat --> acceleracio latera de comfort (maxima)
					//outputs
						// vel_restr -->  vel_restr[0] = [max_k_dist , max_vel_admissible] --> [distancia longitudinal a la que es produeix la maxim curvatura (metres) , maxima velocitat admisible degut a la curvatura mes pronunciada existent]
						//				  vel_restr[1] = [max_col_dist , max_allowed_vel_by_col_or_k] --> [distancia minima on es produeix colisio (per curvatura o obstacle , maxima velocitat quan hi ha colisio o curvatura]	
        
        void compute_best_vp ( C_Traj traj, float our_station, vector <vector<float> > vel_restr, float max_vel_restriction[2], float vel, float max_road_vel, float a_0, float a_ant, float max_decel, float max_accel, float current_t, float delta_t, vector<C_DynObstacle> dyn_obs, float r_col, float lookahead_dist, vector<vector<float> > &best_vp, float &best_vp_cost, float &vp_accel, float &vp_finalvel);
					//inputs
						// trajs --> element de la classe 'C_Traj'
						// our_station --> station on ens trobem
						// vel_restr -->  vel_restr[0] = [max_k_dist , max_vel_admissible] --> [distancia longitudinal a la que es produeix la maxim curvatura (metres) , maxima velocitat admisible degut a la curvatura mes pronunciada existent]
						//				  vel_restr[1] = [max_col_dist , max_allowed_vel_by_col_or_k] --> [distancia minima on es produeix colisio (per curvatura o obstacle , maxima velocitat quan hi ha colisio o curvatura]	
						// max_vel_restriction --> array que indica la maxima velocitat a assolir [m/s] (2on element del array) i a partir de quina distancia longitudinal s'ha d'assolir (metres) (1er element del array)
						// vel --> velocitat porta el cotxe actualment
						// max_road_vel --> maxima velocitat permesa per la carretera
						// a_0 --> acceleracio inicial?¿?
						// a_ant --> acceleracio a la iteracio anterior
						// max_decel --> maxima ?¿?capacitat de desacceleracio
						// max_accel --> maxima ?¿?capacitat acceleracio 
						// current_t --> instant de temps actual [segons] 
						// delta_t --> increments de temps per iteracio
						// vector<C_DynObstacle> dyn_obs --> vector conte info dels obstacles dinamics
						// r_col --> radi comprovacio colisions
						// lookahead_dist --> distancia d'abast de visio
						
					//outputs
						// best_vp --> millor perfil de velocitat --> vector format per 4 vectors 
																	// Punts --> vector amb les diferent posicion a cada instant
																	// Vels --> vector amb les diferent velocitats a cada instant
																	// Accels --> vector amb les diferents acceleracions a cada instant
																	// temps --> vector amb els instants de temps
						// best_vp_cost --> cost associat al millor perfil de velocitat
						// vp_accel --> acceleracio associada al millor perfil de velocitat
						// vp_finalvel --> velocitat final d'aquell perfil de velocitat
						
        void compute_vel_profiles (vector <vector<float> > vel_restr, float max_vel_restriction[2], float vel, float max_vel, float a_0, float a_ant, float max_decel, float max_accel_in, float current_t, float delta_t, float lookahead_dist, vector <vector<vector<float> > > &vel_profiles, vector<float> &c_maxaccels, vector<float> &c_finalvels);
					//inputs
						// vel_restr -->  vel_restr[0] = [max_k_dist , max_vel_admissible] --> [distancia longitudinal a la que es produeix la maxim curvatura (metres) , maxima velocitat admisible degut a la curvatura mes pronunciada existent]
						//	              vel_restr[1] = [max_col_dist , max_allowed_vel_by_col_or_k] --> [distancia minima on es produeix colisio (per curvatura o obstacle , maxima velocitat quan hi ha colisio o curvatura]	
						// max_decel --> es sol entrar com un parametre amb un menys --> '-(maxima_desacceleracio)'			
						// -----mes---
					//outputs
						// vel_profiles --> vector on cada componenr es un perfil de velocitat candidat
											// cada perfil de velocitat conte 4 vectors
												//0	// Punts --> coordenades longitudinals al carril [m]
												//1	// Velocitats --> Velocitats en [m/s] 
												//2	// Accels --> Acceleracions en [m/s^2]
												//3	// temps --> instan de temps [s] en que s'ha de donar que estem a una coordenada longitudinal X, a un velocitat V i amb un acceleracio A. 
						// c_maxaccels --> vector on cada element representa la maxima acceleracio que s'assoleix en el perfil de velocitat que te associat
						// c_finalvels --> vector on cada element representa la velocitat final del perfil de velocitat que te associat
					
		
};
#endif // !__C_TRAJGEN_H_INCLUDED__
