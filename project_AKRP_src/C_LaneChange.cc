#include "C_LaneChange.h" 

using namespace std; 

//constructor 
C_LaneChange::C_LaneChange ( vector <vector<float> > our_center_lane1, vector <vector<float> > other_center_lane1, float resolution1, float sampling_distance1, vector<float> dists1, vector<float> offsets1, C_SamplePoint initial_point1, float target_i_GP1, C_StaticObs static_obs1, float r_col1, float min_turn_r1)
{	
	
	our_center_lane=our_center_lane1;
	other_center_lane = other_center_lane1;
    r_col = r_col1;
    min_turn_r = min_turn_r1;
    initial_point = initial_point1;
    resolution = resolution1;
    sampling_distance = sampling_distance1;
    
	C_Sampling other_sampling_aux (other_center_lane1, resolution1, sampling_distance1, dists1, offsets1, initial_point1, target_i_GP1, static_obs1);
	
	other_sampling=other_sampling_aux;
	
	S_lane_sampling lane_sampling_X;
	other_sampling.sample_lane (r_col, lane_sampling_X); 
		
	S_lane_sampling_comb new_lane_sampling_X;
	other_sampling.traj_combinations( lane_sampling_X, new_lane_sampling_X);
		vector <vector<C_SamplePoint> > traj_points_X = new_lane_sampling_X.traj_points;
		vector <vector<int> > new_feasible_traj_points_X = new_lane_sampling_X.feasible_traj_points;
		vector<float> traj_stations_X = new_lane_sampling_X.traj_stations;
		vector<int> obstruction_here_X = new_lane_sampling_X.obstruction_here;
		float max_feasible_station_X = new_lane_sampling_X.max_feasible_station;
    
		other_lane_blocked = new_lane_sampling_X.lane_blocked;
    
		float first_sampling_index_X = new_lane_sampling_X.first_sampling_index;
		vector<float> offset_combinations_X = new_lane_sampling_X.offset_combinations;
        
	
	if (other_lane_blocked==0)
	{
		C_TrajGen other_traj_gen_aux (new_lane_sampling_X, initial_point1, target_i_GP1, resolution1, static_obs1);
		other_traj_gen= other_traj_gen_aux;
		
		vector<C_Traj> initial_trajs_X;
		other_traj_gen.generate_initial_opt (r_col, min_turn_r, initial_trajs_X); 
		
		vector<C_Traj> rest_trajs_X;
		other_traj_gen.generate_rest (rest_trajs_X); 
		
		vector<C_Traj> trajs_X;
		vector<float> c_lengths_X;
		vector<float> c_maxks_X;
		vector<float> c_maxkdots_X;
		vector<float> c_offsets_X;
		vector<float> c_static_obs_X;
		vector<float> collision_X;
		other_traj_gen.merge(initial_trajs_X, rest_trajs_X, r_col, trajs_X, c_lengths_X, c_maxks_X, c_maxkdots_X, c_offsets_X, c_static_obs_X, collision_X);

		float best_traj_i_X;
		other_traj_gen.best_path (trajs_X, c_lengths_X, c_maxks_X, c_maxkdots_X, c_offsets_X, c_static_obs_X, collision_X, min_turn_r, other_best_traj, other_best_traj_min_cost, other_best_traj_col, best_traj_i_X);

		vector <float> obs_station;
		int no_obstacle;
		static_obs1.get_all_closest_obs_interval( target_i_GP1, (target_i_GP1 + our_center_lane[0].size()), obstacles, obs_station, no_obstacle);

	}
	
}

//funcions membre

int C_LaneChange::lane_change_is_blocked()
{
	return other_lane_blocked;
}


void C_LaneChange::gen_lane_change_trajs (float n, C_Traj our_best_traj, float max_pos[2], float des_sampling_distance, vector<C_Traj> &trajs, vector<float> &c_lengths, vector<float> &c_maxks, vector<float> &c_maxkdots, vector<float> &c_static_obs, vector<float> &collisions, int &lane_change_possible, C_SamplePoint &final_point )
{
	lane_change_possible =1;
	
	if ( other_best_traj_col>0 ) //% Means that the other lane is blocked!!!!!!!!!!!!!!!!!!!!
	{
		lane_change_possible = 0;
	}
	
	float max_station; //  % Maximum station to finish the lane change (index de punt de carril) 
	float other_max_station; // % for not crashing
	if ( (max_pos[0]==-1) && (max_pos[1]==-1) ) // equival a que estigui buit ¿?¿??¿ check
	{
		max_station= our_best_traj.traj[0].size()-1;
		other_max_station= other_best_traj.traj[0].size()-1;
	}
	else
	{
		float dist; // no s'usa
		float dist2; // no s'usa
		Search_closest_point(max_pos[0], max_pos[1], our_best_traj.traj[0], our_best_traj.traj[1], 0, max_station, dist); 
		Search_closest_point(max_pos[0], max_pos[1], other_best_traj.traj[0], other_best_traj.traj[1], 0, other_max_station, dist2); 
	}
	
	// construim el ultim punt de la trajectoria --> sera el corresponent al anterior al corresponent a la de la maxima 'station'
	
	float head_traj  = constrain_angle( atan2( (other_best_traj.traj[1][other_max_station] - other_best_traj.traj[1][other_max_station-1]) , (other_best_traj.traj[0][other_max_station] - other_best_traj.traj[0][other_max_station-1]) ) *180/M_PI, 360.0 ); //calcul orientacio
	final_point.x = other_best_traj.traj[0][other_max_station-1];
    final_point.y = other_best_traj.traj[1][other_max_station-1];
    final_point.k = other_best_traj.traj[2][other_max_station-1];
    final_point.head = head_traj; //% Theta, in deg!
    final_point.s = other_max_station - 1;
	
	// % Initialization of the 'n' trajectories;
	
	C_Traj aux;
	
	for (int i=0; i<n; i++)
	{
		trajs.push_back(aux);
	}
	
	//% COSTS variables: --------------------------------
	//inicialitzem tots els costos a 0
	for (int i=0; i<n; i++)
	{
		c_lengths.push_back(0);
		c_maxks.push_back(0);
		c_maxkdots.push_back(0);
		c_static_obs.push_back(0);
		collisions.push_back(-1); // % For knowing if collision. -1 if not, otherwise it has the target_i of the collision (el -1 indica que no hi ha colisio a aquella station)
	}
	// -------------------------------------------------
	
	C_SamplePoint P_ini;
	C_SamplePoint P_fin;
	if (lane_change_possible==1)
	{
		 float sampling_increment = round(des_sampling_distance/resolution); // indexs (numero de punts) a incrementar en total [-]
		 float increments = round( (max_station - sampling_increment)/n ); // indexs a incrmentar per a cada mostreig [-]
		 
			if (increments > 0) // % OK, normal case
			{
				for (int i=0; i<n; i++) //% CHANGE TRAJECTORIES GENENERATION --> Generem cada una de les trajectories candidates de trasllat al altre carril, de forma que cada una te el punt final es una 'station' mes allunyada que l'anterior 
				{
					if (i==0) // % The first time
					{
						P_ini=initial_point;
					}
					else 
					{
						float index = (increments*i)-1;
						if ( index > max_station -1)
						{
							index = max_station - 1;
						}
						
						float head_traj  = constrain_angle( atan2( (our_best_traj.traj[1][index + 1] - our_best_traj.traj[1][index]) , (our_best_traj.traj[0][index + 1] - our_best_traj.traj[0][index]) ) *180/M_PI, 360.0 ); //calcul orientacio

						 P_ini.x = our_best_traj.traj[0][index];
                         P_ini.y = our_best_traj.traj[1][index];
                         P_ini.k = our_best_traj.traj[2][index];
                         P_ini.head = head_traj; //% Theta, in deg!
                         P_ini.s = index;
					}
					
					if ( i==(n-1) ) // % The last time
					{
						P_fin = final_point;
					}
					else
					{
						float index = ((increments*i)-1)+sampling_increment;
						
						if (index > (other_max_station-1))
						{
							index = other_max_station - 1;
						}
						
						float head_traj  = constrain_angle( atan2( (other_best_traj.traj[1][index + 1] - other_best_traj.traj[1][index]) , (other_best_traj.traj[0][index + 1] - other_best_traj.traj[0][index]) ) *180/M_PI, 360.0 ); //calcul orientacio
                        
                        P_fin.x = other_best_traj.traj[0][index];
                        P_fin.y = other_best_traj.traj[1][index];
                        P_fin.k = other_best_traj.traj[2][index];
                        P_fin.head = head_traj; //% Theta, in deg!
                        P_fin.s = index;
					}
					
					vector<float> Px;
					vector<float> Py;
					float dist;
					vector<float> Pk;
					
					G2_Spline(P_ini.x, P_ini.y, P_ini.k, P_ini.head*M_PI/180, P_fin.x, P_fin.y, P_fin.k, P_fin.head*M_PI/180, 5, resolution, Px, Py, dist, Pk);

					// Generacio del vector de trajectories candidates (definies pel punt inicial que tenen en comu i el final (el qual anira variant segons el candidat), les punts de les coordenades que les defineixen, etc...)
					
					if (i==0) //% The first time
					{
						
						float ini_point_sampling_coords1[2]={-1,-1}; //--> -1 -1 ja que estem indicant un punt de fora de la matriu que representa la discretitzacio espaial del carril
						float end_point_sampling_coords1[2]={-1,-1};
						
						// contruccio trajectoria --> Primer posem la trajectoria anterior i despres la trajectoria fins el punt inicial on començara el canvi de carril
						vector <vector<float> > traj_aux;
						
						traj_aux.push_back(Px);
						traj_aux.push_back(Py);
						traj_aux.push_back(Pk);
						
						for (int j=P_fin.s; j<=(int)final_point.s; j++) 
						{
							traj_aux[0].push_back( other_best_traj.traj[0][j] );
							traj_aux[1].push_back( other_best_traj.traj[1][j] );
							traj_aux[2].push_back( other_best_traj.traj[2][j] );
						}
						
						C_Traj aux ( ini_point_sampling_coords1, end_point_sampling_coords1, traj_aux, dist+((other_max_station - P_fin.s+1)*resolution), resolution);
						
						trajs[i]=aux;
						
					}
					else if ( i==(n-1) ) // The last time
					{
						float ini_point_sampling_coords1[2]={-1,-1}; //--> -1 -1 ja que estem indicant un punt de fora de la matriu que representa la discretitzacio espaial del carril
						float end_point_sampling_coords1[2]={-1,-1};
						
						// contruccio trajectoria
						vector <vector<float> > traj_aux;
						vector<float> traj_x_aux;
						vector<float> traj_y_aux;
						vector<float> traj_k_aux;
						
						for (int j=0; j<=(int)P_ini.s; j++) // primer tram fins a P.ini
						{
							traj_x_aux.push_back( our_best_traj.traj[0][j] );
							traj_y_aux.push_back( our_best_traj.traj[1][j] );
							traj_k_aux.push_back( our_best_traj.traj[2][j] );
						}
						
						traj_aux.push_back(traj_x_aux);
						traj_aux.push_back(traj_y_aux);
						traj_aux.push_back(traj_k_aux);
						
						for (int j=0; j<(int)Px.size(); j++) 
						{
							traj_aux[0].push_back( Px[j] );
							traj_aux[1].push_back( Py[j] );
							traj_aux[2].push_back( Pk[j] );
						}
						
						C_Traj aux ( ini_point_sampling_coords1, end_point_sampling_coords1, traj_aux, dist+((P_ini.s+1)*resolution), resolution);
						
						trajs[i]=aux;
						
					}
					else // trajectoria intermitja
					{
						float ini_point_sampling_coords1[2]={-1,-1}; //--> -1 -1 ja que estem indicant un punt de fora de la matriu que representa la discretitzacio espaial del carril
						float end_point_sampling_coords1[2]={-1,-1};
						
						// contruccio trajectoria
						vector <vector<float> > traj_aux;
						vector<float> traj_x_aux;
						vector<float> traj_y_aux;
						vector<float> traj_k_aux;
						
						for (int j=0; j<=(int)P_ini.s; j++) // primer tram fins a P.ini
						{
							traj_x_aux.push_back( our_best_traj.traj[0][j] );
							traj_y_aux.push_back( our_best_traj.traj[1][j] );
							traj_k_aux.push_back( our_best_traj.traj[2][j] );
						}
						
						traj_aux.push_back(traj_x_aux);
						traj_aux.push_back(traj_y_aux);
						traj_aux.push_back(traj_k_aux);
						
						for (int j=0; j<(int)Px.size(); j++) 
						{
							traj_aux[0].push_back( Px[j] );
							traj_aux[1].push_back( Py[j] );
							traj_aux[2].push_back( Pk[j] );
						}
						
						for (int j=P_fin.s; j<=(int)final_point.s; j++) 
						{
							traj_aux[0].push_back( other_best_traj.traj[0][j] );
							traj_aux[1].push_back( other_best_traj.traj[1][j] );
							traj_aux[2].push_back( other_best_traj.traj[2][j] );
						}
						
						C_Traj aux ( ini_point_sampling_coords1, end_point_sampling_coords1, traj_aux, dist+((other_max_station-P_fin.s+P_ini.s+1)*resolution), resolution);
						
						trajs[i]=aux;
					}
					
					//% Costs computation:

                    c_lengths[i] = trajs[i].len;
                    c_maxks[i] = trajs[i].maxk;
                    c_maxkdots[i] = trajs[i].maxkdot;
					
					float c_static_obs_aux;	
					float col_index;				
					trajs[i].compute_cost_static_obs(obstacles, r_col, c_static_obs_aux, col_index); //calcula es cost estatic total asociat de la trajectoria respecte els diferent obstacles que hi han
					c_static_obs[i]=c_static_obs_aux;
					
					if ( col_index > -1 )
					{
						collisions[i] = col_index;
					}
				
				}
				
			}
			else // % increments <= 0. This means that the obstacle is too close to us to generate n trajs
			{
				float ini_point_sampling_coords1[2]={-1,-1}; //--> -1 -1 ja que estem indicant un punt de fora de la matriu que representa la discretitzacio espaial del carril
				float end_point_sampling_coords1[2]={-1,-1};
						
				// contruccio trajectoria --> Primer posem la trajectoria anterior i despres la trajectoria fins el punt inicial on començara el canvi de carril
				vector <vector<float> > traj_aux;
						
				for (int j=0; j<=(int)(other_max_station-1); j++) 
				{
					traj_aux[0].push_back( other_best_traj.traj[0][j] );
					traj_aux[1].push_back( other_best_traj.traj[1][j] );
					traj_aux[2].push_back( other_best_traj.traj[2][j] );
				}
						
				C_Traj traj1_aux ( ini_point_sampling_coords1, end_point_sampling_coords1, traj_aux, (other_max_station+1)*resolution, resolution);
						
				for (int i=0; i<n; i++) //  % Trajectories generation
				{
					trajs[i] = traj1_aux;
					
					// % Costs computation:

					c_lengths[i] = trajs[i].len;
                    c_maxks[i] = trajs[i].maxk;
                    c_maxkdots[i] = trajs[i].maxkdot;
                    
                    float c_static_obs_aux;	
					float col_index;				
					trajs[i].compute_cost_static_obs(obstacles, r_col, c_static_obs_aux, col_index); //calcula es cost estatic total asociat de la trajectoria respecte els diferent obstacles que hi han
					c_static_obs[i]=c_static_obs_aux;
					
					if ( col_index > -1 )
					{
						collisions[i] = col_index;
					}
				}
			}
			
			//% Check collisions: llocs on estan les colisions
			
			vector <float> col_indexs;
			
			for (int i=0; i<(int)collisions.size(); i++) //  % Trajectories generation
			{
				if (collisions[i]>-1)
				{
					col_indexs.push_back(i);
				}
			}
			
			int num_cols = col_indexs.size();
			
			if (num_cols==n)
			{
				lane_change_possible = 0; //% All trajs crash! --> no es possible canviar de carril
			}
	} 
}


void C_LaneChange::best_lane_change_traj (vector<C_Traj> trajs, vector<float> c_lengths, vector<float> c_maxks, vector<float> c_maxkdots, vector<float> c_static_obs, vector<float> collisions, float comfort_a_lat, float max_vel_restriction[2], float vel, float max_road_vel, float a_0, float a_ant, float max_decel, float max_accel, float current_t, float delta_t, vector<C_DynObstacle> dyn_obs, C_Traj &best_traj, float &best_traj_cost, float &best_traj_col, vector<vector<float> > &best_traj_vp, float &best_traj_vp_cost, float &best_vp_accel, float &best_vp_finalvel)
{
	float lookahead_dist = other_best_traj.len; //% Length in [m] of the best traj in the other lane
	
	int num_trajs = trajs.size();
	
	vector <vector<vector<float> > > vel_restrictions; //vector de matrius 2x2, indicant :
													// vel_restr -->  vel_restr[0] = [max_k_dist , max_vel_admissible] --> [distancia longitudinal a la que es produeix la maxim curvatura (metres) , maxima velocitat admisible degut a la curvatura mes pronunciada existent]
													// vel_restr[1] = [max_col_dist , max_allowed_vel_by_col_or_k] --> [distancia minima on es produeix colisio (per curvatura o obstacle , maxima velocitat quan hi ha colisio o curvatura]	
	vector <vector<vector<float> > > best_vp; // vectors amb els perfils de velocitats, els quals estan formats per
													// 4 vecrtors 
														// Punts --> vector amb les diferent posicion a cada instant
														// Vels --> vector amb les diferent velocitats a cada instant
														// Accels --> vector amb les diferents acceleracions a cada instant
														// temps --> vector amb els instants de temps
	vector<float> best_vp_min_cost; // vector que conte els costs minims de cada trajectoria												
	vector<float> vp_accel; // accelarations used in each velocity profile generated
	vector<float> vp_finalvel; // final velocity reached in each velocty profile
	
	// Cumputation of the best velocity profile for each candidate path --> we have the same number of velocity profiles as number of paths
	
	for (int i=0; i<num_trajs; i++) 
	{
		vector <vector<float> > vp_restrict_aux;
		other_traj_gen.compute_vel_static_restrictions ( trajs[i], 0, collisions[i], min_turn_r, comfort_a_lat, vp_restrict_aux);
		vel_restrictions.push_back(vp_restrict_aux);
		
		
		vector<vector<float> > best_vp_aux; 
		float best_vp_cost_aux; 
		float vp_accel_aux; 
		float vp_finalvel_aux;
		other_traj_gen.compute_best_vp ( trajs[i], 0, vel_restrictions[i], max_vel_restriction, vel, max_road_vel, a_0, a_ant, max_decel, max_accel, current_t, delta_t, dyn_obs, r_col, lookahead_dist, best_vp_aux, best_vp_cost_aux, vp_accel_aux, vp_finalvel_aux);
		
		best_vp.push_back(best_vp_aux);
		best_vp_min_cost.push_back(best_vp_cost_aux);
		vp_accel.push_back(vp_accel_aux);
		vp_finalvel.push_back(vp_finalvel_aux);
	}
	
	// % COSTS normalization: --------------------------------
	
	vector<float> c_lengths_norm;
	vector<float> c_maxks_norm;
	vector<float> c_maxkdots_norm;
	
	//inicialitzacio costos totals a 0
	for (int i=0; i<(int)trajs.size(); i++)
	{
		c_lengths_norm.push_back(0);
		c_maxks_norm.push_back(0);
		c_maxkdots_norm.push_back(0);
	}
	
	vector<float> total_cost; 
	float total_cost_aux;
	
	for (int i=0; i<(int)c_lengths.size(); i++) 
	{
		c_lengths_norm[i]=c_lengths[i]/(other_best_traj.len); // cost=(longitud trajectoria/longitud de la trajectoria d'offset 0)
		c_maxks_norm[i]=c_maxks[i]/(1/min_turn_r);
		c_maxkdots_norm[i]=c_maxkdots[i]/(1/min_turn_r);
		
		// for each candidate path it is computed the (total cost= static_cost + dynamic_cost )--> to choose the best path to change the lane
		total_cost_aux= c_lengths_norm[i]+c_maxks_norm[i]+c_maxkdots_norm[i]+c_static_obs[i] + best_vp_min_cost[i];
		total_cost.push_back(total_cost_aux);
	}
	
	// Choice of the best trajectory with the best velocity profile
	
	best_traj_cost=min_value(total_cost);
	float traj_i;
	
	for (int i=0; i<(int)total_cost.size(); i++)
	{
		if (total_cost[i]==best_traj_cost)
		{
			traj_i=i; 
			break;
		}
	}
	
	best_traj=trajs[traj_i]; // optimal trajectory choosen (minimum cost)
	best_traj_col=collisions[traj_i]; // point put which trajectories produce colision
	
	best_traj_vp = best_vp[traj_i];
    best_traj_vp_cost = best_vp_min_cost[traj_i]; 
    best_vp_accel = vp_accel[traj_i];
    best_vp_finalvel = vp_finalvel[traj_i];

}



void C_LaneChange::gen_avoidance ( float n, C_Traj our_best_traj, C_Traj best_traj_X, float max_pos[2], vector<C_Traj> &trajs, vector<float> &c_lengths, vector<float> &c_maxks, vector<float> &c_maxkdots, vector<float> &c_static_obs, vector<float> &collisions, int &lane_change_possible)  
{
	C_SamplePoint final_point;
	gen_lane_change_trajs (n, our_best_traj, max_pos, sampling_distance, trajs, c_lengths, c_maxks, c_maxkdots, c_static_obs, collisions, lane_change_possible, final_point);

	float station_av;
	float points_dist;
	Search_closest_point( final_point.x, final_point.y, best_traj_X.traj[0], best_traj_X.traj[1], 0, station_av, points_dist); 

	if ( points_dist>=resolution ) // % The 2 paths are a little discontinuous due to an static obstacle that induces lateral offset
	{
		float initial_index_other;
		float points_dist2; // no s'usa
		Search_closest_point( final_point.x, final_point.y, other_best_traj.traj[0], other_best_traj.traj[1], 0, initial_index_other, points_dist2); 

		float index;
		float points_dist; // no s'usa

		for (int i=0; i<(int)(other_best_traj.traj[0].size()-initial_index_other); i++) //  % Trajectories generation
		{
			float point1[2];
			point1[0]=other_best_traj.traj[0][initial_index_other+i];
			point1[1]=other_best_traj.traj[1][initial_index_other+i];
			
			
			Search_closest_point( point1[0], point1[1], other_best_traj.traj[0], other_best_traj.traj[1], station_av, index, points_dist); 
			
			if (points_dist<(resolution/4))
			{
				station_av = index;
				break;
			}
		}
		
		float head_traj  = constrain_angle( atan2( (best_traj_X.traj[1][index+1] - best_traj_X.traj[1][index]) , (best_traj_X.traj[0][index+1] - best_traj_X.traj[0][index]) ) *180/M_PI, 360.0 ); //calcul orientacio

		vector<float> Px;
		vector<float> Py;
		float dist;
		vector<float> Pk;
					
		G2_Spline(final_point.x, final_point.y, final_point.k, final_point.head*M_PI/180, best_traj_X.traj[0][index], best_traj_X.traj[1][index], best_traj_X.traj[2][index], head_traj*M_PI/180, 5, resolution, Px, Py, dist, Pk);
		
		for (int i=0; i<n; i++) //  % Trajectories generation
		{
			float ini_point_sampling_coords1[2]={-1,-1}; //--> -1 -1 ja que estem indicant un punt de fora de la matriu que representa la discretitzacio espaial del carril
			float end_point_sampling_coords1[2]={-1,-1};
						
			// contruccio trajectoria 
			vector <vector<float> > traj_aux;
						
			traj_aux.push_back(trajs[i].traj[0]);
			traj_aux.push_back(trajs[i].traj[1]);
			traj_aux.push_back(trajs[i].traj[2]);
		
			
			for (int j=0; j<(int)Px.size(); j++) //  % Trajectories generation
			{
				traj_aux[0].push_back(Px[j]);
				traj_aux[1].push_back(Py[j]);
				traj_aux[2].push_back(Pk[j]);
			}
			
			
			C_Traj aux ( ini_point_sampling_coords1, end_point_sampling_coords1, traj_aux, trajs[i].len + dist, resolution);
			trajs[i]=aux;
		}
	}
	
	
	float ini_point_sampling_coords1[2]={-1,-1};
	float end_point_sampling_coords1[2]={-1,-1};
	
	vector <vector<float> > traj_aux;
	vector<float> traj_x_aux;
	vector<float> traj_y_aux;
	vector<float> traj_k_aux;
	
	for (float i=station_av; i<best_traj_X.traj[0].size(); i++) //  % Trajectories generation
	{
		traj_x_aux.push_back(best_traj_X.traj[0][i]);
		traj_y_aux.push_back(best_traj_X.traj[1][i]);
		traj_k_aux.push_back(best_traj_X.traj[2][i]);
	}
	
	traj_aux.push_back(traj_x_aux);
	traj_aux.push_back(traj_y_aux);
	traj_aux.push_back(traj_k_aux);
	
	float dist= (traj_aux[0].size()+1)*resolution; // longitud de la trajectoria [metres]
	
	C_Traj return_traj ( ini_point_sampling_coords1, end_point_sampling_coords1, traj_aux, dist, resolution);
	
	
	for (int i=0; i<n; i++) 
	{
		vector <vector<float> > new_traj;
		
		new_traj.push_back(trajs[i].traj[0]);
		new_traj.push_back(trajs[i].traj[1]);
		new_traj.push_back(trajs[i].traj[2]);
		
		for (float j=0; j<return_traj.traj[0].size(); j++) 
		{
			new_traj[0].push_back(return_traj.traj[0][j]);
			new_traj[1].push_back(return_traj.traj[1][j]);
			new_traj[2].push_back(return_traj.traj[2][j]);
		}
		
		float new_dist = trajs[i].len + return_traj.len;
		
		C_Traj aux ( ini_point_sampling_coords1, end_point_sampling_coords1, new_traj, new_dist, resolution); 
		
		// afegim al vector de trajectories
		trajs[i]=aux;
		
		// Costs computation
		c_lengths[i] = trajs[i].len;
        c_maxks[i] = trajs[i].maxk;
        c_maxkdots[i] = trajs[i].maxkdot;
        
        float c_static_obs_aux;	
		float col_index;				
		
		trajs[i].compute_cost_static_obs(obstacles, r_col, c_static_obs_aux, col_index); //calcula es cost estatic total asociat de la trajectoria respecte els diferent obstacles que hi han
		c_static_obs[i]=c_static_obs_aux;
        
        if (col_index>-1)
        {
			collisions[i]=col_index;
		}
		else
		{
			collisions[i]=0;
		}
		
	}
	
	// % Check collisions:
	
	vector <float> col_indexs;
			
	for (int i=0; i<(int)collisions.size(); i++) //  % Trajectories generation
	{
		if (collisions[i]>-1)
		{
			col_indexs.push_back(i);
		}
	}
			
	int num_cols = col_indexs.size();
			
	if (num_cols==n)
	{
		lane_change_possible = 0; //% All trajs crash! --> no es possible canviar de carril
	}
	else
	{
		lane_change_possible = 1;
	}

}  


void C_LaneChange::gen_dyn_avoidance ( float n, C_Traj our_best_traj, vector<float> dists, vector<float> offsets, C_StaticObs static_obs, vector<C_Traj> &trajs, vector<float> &c_lengths, vector<float> &c_maxks, vector<float> &c_maxkdots, vector<float> &c_static_obs, vector<float> &collisions, int &lane_change_possible)
{
	float half_station = round( other_sampling.final_station/2.0 );
	
	float max_pos[2] = {other_best_traj.traj[0][half_station] , other_best_traj.traj[1][half_station]}; // X and Y coordenates of the las position --> % Is the half point of the other lane best traj
	
	
	// computation of the candidate trajectories
	vector<C_Traj> trajs1;
	vector<float> c_lengths1;
	vector<float> c_maxks1;
	vector<float> c_maxkdots1;
	vector<float> c_static_obs1;
	vector<float> collisions1;
	int lane_change_possible1;
	C_SamplePoint final_point1;
			
	gen_lane_change_trajs (n, our_best_traj, max_pos, sampling_distance, trajs1, c_lengths1, c_maxks1, c_maxkdots1, c_static_obs1, collisions1, lane_change_possible1, final_point1);
	
	//  % Trajectories generation from half station up to the end
	vector <vector<float> > coming_center_lane;
	vector<float> coming_center_x_lane;
	vector<float> coming_center_y_lane;
	vector<float> coming_center_k_lane;
	
	for (int i=half_station; i<(int)other_best_traj.traj[0].size(); i++) 
	{
		coming_center_x_lane.push_back(other_best_traj.traj[0][i]);
		coming_center_y_lane.push_back(other_best_traj.traj[1][i]);
		coming_center_k_lane.push_back(other_best_traj.traj[2][i]);
	}
	coming_center_lane.push_back(coming_center_x_lane);
	coming_center_lane.push_back(coming_center_y_lane);
	coming_center_lane.push_back(coming_center_k_lane);
	
	
	vector <vector<float> > coming_other_center_lane;
	vector<float> coming_other_center_x_lane;
	vector<float> coming_other_center_y_lane;
	vector<float> coming_other_center_k_lane;
	
	for (int i=half_station; i<(int)our_best_traj.traj[0].size(); i++) 
	{
		coming_other_center_x_lane.push_back(our_best_traj.traj[0][i]);
		coming_other_center_y_lane.push_back(our_best_traj.traj[1][i]);
		coming_other_center_k_lane.push_back(our_best_traj.traj[2][i]);
	}
	coming_other_center_lane.push_back(coming_other_center_x_lane);
	coming_other_center_lane.push_back(coming_other_center_y_lane);
	coming_other_center_lane.push_back(coming_other_center_k_lane);
	
	C_SamplePoint ini_point;
	ini_point = final_point1;
	
	float target_i_GP=other_sampling.initial_point_GP_path_i + half_station;
	
	C_LaneChange second_lane_change ( coming_center_lane, coming_other_center_lane, resolution, sampling_distance, dists, offsets, ini_point, target_i_GP, static_obs, r_col, min_turn_r);
	
	int coming_lane_blocked=second_lane_change.lane_change_is_blocked(); 
	
	float max_pos_empty[2]={-1,-1};
	vector<C_Traj> trajs2;
	vector<float> c_lengths2;
	vector<float> c_maxks2;
	vector<float> c_maxkdots2;
	vector<float> c_static_obs2;
	vector<float> collisions2;
	int lane_change_possible2;
	C_SamplePoint final_point2;
	
		
	if (coming_lane_blocked==0) //no blocked
	{
		float ini_point_sampling_coords1[2]={-1,-1};
		float end_point_sampling_coords1[2]={-1,-1};
		
		C_Traj coming_center_lane_best_traj ( ini_point_sampling_coords1, end_point_sampling_coords1, coming_center_lane, (coming_center_lane[0].size()*resolution), resolution);
		
		second_lane_change.gen_lane_change_trajs (n, coming_center_lane_best_traj, max_pos_empty, sampling_distance, trajs2, c_lengths2, c_maxks2, c_maxkdots2, c_static_obs2, collisions2, lane_change_possible2, final_point2);
	}
	
	float new_n=n*n;
	
	//inicialitzem tots els costos a 0
	for (int i=0; i<new_n; i++)
	{
		c_lengths.push_back(0);
		c_maxks.push_back(0);
		c_maxkdots.push_back(0);
		c_static_obs.push_back(0);
		collisions.push_back(-1); // % For knowing if collision. -1 if not, otherwise it has the target_i of the collision (el -1 indica que no hi ha colisio a aquella station)
	}
	
	
	
	//Trajectory combination
	
	float count = 0;
	
	for (int i=0; i<n; i++)
	{
		for (int j=0; j<n; j++)
		{
			vector <vector<float> > new_traj;
			new_traj.push_back(trajs1[i].traj[0]);
			new_traj.push_back(trajs1[i].traj[1]);
			new_traj.push_back(trajs1[i].traj[2]);
			
			for (int k=0; k<(int)trajs2[j].traj[0].size(); k++)
			{
				new_traj[0].push_back(trajs2[j].traj[0][k]);
				new_traj[1].push_back(trajs2[j].traj[1][k]);
				new_traj[2].push_back(trajs2[j].traj[2][k]);
			}
			
			float ini_point_sampling_coords1[2]={-1,-1};
			float end_point_sampling_coords1[2]={-1,-1};
			
			float new_dist= trajs1[i].len + trajs2[j].len;
			
			C_Traj traj_aux( ini_point_sampling_coords1, end_point_sampling_coords1, new_traj, new_dist, resolution);
			trajs.push_back(traj_aux);
			
			// % Costs computation
			c_lengths[count] = trajs[count].len;
            c_maxks[count] = trajs[count].maxk;
            c_maxkdots[count] = trajs[count].maxkdot;

			float c_static_obs_aux;	
			float col_index;				
			trajs[i].compute_cost_static_obs(obstacles, r_col, c_static_obs_aux, col_index); //calcula es cost estatic total asociat de la trajectoria respecte els diferent obstacles que hi han
			c_static_obs[i]=c_static_obs_aux;
			
			if (col_index>0) // % If there is collision
			{
				collisions[count] = col_index;
			}
			else
			{
				collisions[count] = 0;
			}
			count = count + 1;
		}
	}
	
	//% Check collisions
	vector <float> col_indexs;
			
	for (int i=0; i<(int)collisions.size(); i++) //  % Trajectories generation
	{
		if (collisions[i]>-1)
		{
			col_indexs.push_back(i);
		}
	}
			
	int num_cols = col_indexs.size();
			
	if (num_cols==new_n)
	{
		lane_change_possible = 0; //% All trajs crash! --> no es possible canviar de carril
	}
	else
	{
		lane_change_possible = 1;
	}
} 

