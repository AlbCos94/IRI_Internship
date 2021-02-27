#include "C_Sampling.h" 

using namespace std; 

//constructor 
C_Sampling::C_Sampling ( vector <vector<float> > center_lane1, float resolution1, float sampling_dist1, vector <float > dists1, vector <float > offsets1, C_SamplePoint initial_point1, float initial_point_GP_path_i1, C_StaticObs static_obs1)
{
	center_lane=center_lane1;
	final_station=center_lane1[0].size()-2; //numero de index del punt final del tram de carril a tractat ('-2' For preventing overflow when computing an i+1)
	resolution=resolution1;
	
	for (int i=0; i<(float)dists1.size(); i=i+1)
	{
		stations.push_back( round( (sampling_dist1*dists1[i])/resolution ) -1 ); // pasem els metres de la discrtitzcio longitudinal, a indexos de punts del carril; el -1 es degut a que comencem a indexar pel '0'
	} 
	
	offsets=offsets1;
	initial_point=initial_point1;
	initial_point_GP_path_i=initial_point_GP_path_i1;
	static_obs=static_obs1;
	
}

//default constructor 
C_Sampling::C_Sampling ()
{
	vector <vector<float> > center_lane_aux_aux;
	vector <float> center_lane_aux;
	center_lane_aux.push_back(0);
	center_lane_aux_aux.push_back(center_lane_aux); 
	
	center_lane=center_lane_aux_aux;
	final_station=-1;
	resolution=0;
	
	vector <float > stations_aux;
	stations_aux.push_back(0);
	stations=stations_aux;
	
	vector <float > offsets_aux;
	offsets_aux.push_back(0);
	offsets=offsets_aux;
	
	C_SamplePoint aux;
	initial_point=aux;
	initial_point_GP_path_i=-1;
	
	C_StaticObs aux1;
	
	static_obs=aux1;
	
	
}


//funcions membres de la classe 
void C_Sampling::sample (float initial_point_GP_path_i, vector <float > desired_stations, float r_col, vector <vector<C_SamplePoint> > &traj_points, vector <vector<int> > &feasible_traj_points, float &max_feasible_station, int &lane_blocked) // funcio per mostrejar
{
	int flag_obstacle_inside=0; // es posa a 0 quan el obstacle associat a una station no obstaculitza a cap offset
	lane_blocked=0; // % Flag to indicate if the lane is totally blocked by an obstacle (cas en que valdra 1) --> per defecte fem que val 0
	
	float desired_station = desired_stations.back(); // index de la ultima 'station'
	
	// Retallem carril, si la 'station' a la que volem arrivar es posterior a la ultima del carril tractat
	if (desired_station > final_station) //% For preventing going out of bounds in our center_lane
	{
		desired_station=final_station; // com a molt arrivarem a la 'station' maxima del carril tractat
		
		// % Cutting the stations vector (el tallem fins a la estacio final, si la estacio desitjada esta mes allunyada d'aquesra) --> el nostre tram no es tan llarg
		vector <int > feas_stations; // vector de 1s i 0s ( 1s indica les 'stations'  menors a la 'station' final
		int max_feas_index=0; // index del ultim element que es mes petit a la 'final station'
		for (int i=0; i<(float)desired_stations.size(); i=i+1)
		{
			if (desired_stations[i]<final_station)
			{
				feas_stations.push_back(1);
				max_feas_index=i;
			}
			else
			{
				feas_stations.push_back(0);
			}
		}
		vector <float > desired_stations_aux;
		for (int i=0; i<max_feas_index; i=i+1)
		{
			desired_stations_aux.push_back(desired_stations[i]);
		}
		desired_stations_aux.push_back(desired_station); //% Also append the last station
		
		desired_stations=desired_stations_aux;
	}
	
	float max_desired_GP_path_i = initial_point_GP_path_i+desired_station; // ultim index en que es mostreja el carril
	
	vector <C_Obstacle> closest_obs; // vector amb els obstacles que conte el tram de carril
	vector <float> obs_stations; // vector amb les stations corresponents on existeix un obstacle o mes aprop
	int no_obstacle; // valdra 1 quan es detecti algun obstacle, sino valdra 0 
	static_obs.get_all_closest_obs_interval( initial_point_GP_path_i, max_desired_GP_path_i, closest_obs, obs_stations, no_obstacle); // retorna un vector de tots els obstacles visibles i ordenats de mes aprop a més lluny i dins un interval de 'stations' indicat pel valors de 'station' major i el valor de 'station' menor  
	int num_obs=0; // numero obstacles tenim
	
	if(no_obstacle==1)
	{
		num_obs=closest_obs.size();
	}
	
	// Endpoints generation algorithm (pg 42 TFM)
	// relacio dels obstacles amb les 'stations' del tram del carril
	
	//variables globals
	float obs_station; // 'station' mes propera al objecte
	float obs_dist; // no s'empra (distancia del obstacle a la 'station')
	vector <float> used_offsets; //vector on s'emmagazemaran els indexs dels offset del vector de ofsets que es poden emprar (perque no tenen un obstacle que ho impedeixi)
	
	if (num_obs>0)
	{
		for (int o=0; o<num_obs; o=o+1) // PER CADA OBSTACLE
		{			
			Search_closest_point(closest_obs[o].pos[0], closest_obs[o].pos[1], center_lane[0], center_lane[1], 0, obs_station, obs_dist); 
			
			if (obs_station>desired_station)
			{
				obs_station = desired_station; // com a molt es relacionara amb la ultima 'station'
			}
			
			int num_obs_same_station; // numero de obstacles en la mateixa estacio
			if (o <= num_obs-2)
			{
				if ( obs_stations[o] == obs_stations[o+1] ) // el obstacle actual i el proxim es troben a la mateixa estacio
				{
					num_obs_same_station=2;
				}
				else
				{
					num_obs_same_station=1;
				}
			}
			else
			{
				num_obs_same_station=1;
			}
			
			int flag_obstacle_inside=1; // considerem que hi ha almenys un obstacle que ens obstrueix el carril
			// variable global --> vector <float> used_offsets; //vector on s'emmagazemaran els indexs dels offset del vector de ofsets que es poden emprar (perque no tenen un obstacle que ho impedeixi)
			vector <C_SamplePoint> end_points; // vector que conte elements de la classe C_SamplePoint que representen els 'end_points' de la 'station' asociada al obstacle
			float head_traj;
			
			head_traj= constrain_angle( atan2( (center_lane[1][obs_station+1] - center_lane[1][obs_station]) , (center_lane[0][obs_station+1] - center_lane[0][obs_station])  )*(180/M_PI) , 360.0 ); //calcul orientacio
			
			for (int j=0; j<(int)offsets.size(); j++)
			{
				C_SamplePoint aux; // End point de la 'station' asociada al obstacle
				aux.x = center_lane[0][obs_station]+offsets[j]*cos((head_traj*M_PI/180.0) + M_PI/2); // coord X del obstacle
				aux.y = center_lane[1][obs_station]+offsets[j]*sin((head_traj*M_PI/180.0) + M_PI/2); // coord Y del obstacle
				aux.k = 1 / ( (1/center_lane[2][obs_station])-offsets[j] );
				aux.head = head_traj; // 'theta' [graus]
				aux.s = obs_station; // aquells punt tindran associada lamateixa estacio
				
				end_points.push_back( aux ); // afegim el 'end_point' a la llista de endpoints d'aquella 'station' 
				
				if (num_obs_same_station == 1) //nomes hi ha un obstacle a aquella 'station'
				{
					if ( check_col_sample_point(aux, r_col, closest_obs[o]) == 0 ) // no hi ha colisio en aquell offset al el obstacle d'aquella 'station' / '-1' a 'closest_obs[o-1]' els indexs de 'o' comencen en 1 i no en 0
					{
						used_offsets.push_back(j); // es podra emprar el offset pertanyent a aquell index com a 'end_point'
					}
				}
				else if (num_obs_same_station == 2) // dos obstacle en la mateiza 'station' 
				{
					if ( (check_col_sample_point(aux, r_col, closest_obs[o])==0) && (check_col_sample_point(aux, r_col, closest_obs[o+1])==0) ) 
					{
						used_offsets.push_back(j); 
					}				
				}
			} //NO ES PODEN TENIR MES DE DOS OBSTACLES A LA MATEIXA 'STATION'
			
			//Cas en que esta completament bloquejat el carril (cap offset disponible)
			// If it's completely blocked, we only need the center as a good trajectory, so only pick the central offset
			if (used_offsets.empty()==1) // no hi ha cap offset que no estigui obstruit per un obstacle (cap index del vector de offsets disponible)
			{
				lane_blocked=1;
				used_offsets.push_back(ceil(offsets.size()/2)-1); 
			}
			else
			{
				if (used_offsets.size() == offsets.size()) // cap offset es pbstruit per l'obstacle
				{
					flag_obstacle_inside=0;  // el obstacle es pot menystenir, ja que no obstaculitza l'acces a cap offset
				}
			}
			
			if (flag_obstacle_inside==1)
			{
				break; // acabem aqui, ja em detectat un obstacle i aquest es el que considerem que crea la obstruccio
			}
		}
	}
	
	// Mostreig quan NO hi han obstacle a la discretitzacio espaial feta 
	if (flag_obstacle_inside == 0) // no hi cap obstacle a l'area discretitzada ( o cap afecta a les trajectories9
	{
		//Elements del output
		//vector <vector<int> > feasible_traj_points; // matriu d'elements de 1s o 0s que representa si la posocio amb un offset i station determinada esta ocuoda (0) o no per un obstacle (1) 
		//vector <vector<C_SamplePoint> > traj_points; // matriu d'elements C_SamplePoint que representa la dsicretitzacio de tota l'area d'actuacio 
		
		float current_s;
		float head_traj;
		
		for (int i=0; i<(int)desired_stations.size(); i++) // PER A CADA 'STATION' (COLUMNA)
		{
			current_s=desired_stations[i];
			head_traj=constrain_angle( atan2( (center_lane[1][current_s+1] - center_lane[1][current_s]) , (center_lane[0][current_s+1] - center_lane[0][current_s])  )*(180/M_PI) , 360.0 ); //calcul orientacio
			
			vector <C_SamplePoint> traj_points_aux;
			vector <int> feasible_traj_points_aux;
			
			for (int j=0; j<(int)offsets.size(); j++) // PER A CADA OFFSET (FILA)
			{
				//construim cada un dels elements de la matriu 'traj_points' --> elements de la clase C_SamplePoint
				C_SamplePoint aux; // End point de la 'station' asociada al obstacle
				aux.x = center_lane[0][current_s]+offsets[j]*cos((head_traj*M_PI/180.0) + M_PI/2); // coord X del obstacle
				aux.y = center_lane[1][current_s]+offsets[j]*sin((head_traj*M_PI/180.0) + M_PI/2); // coord Y del obstacle
				aux.k = 1 / ( (1/center_lane[2][current_s])-offsets[j] );
				aux.head = head_traj; // 'theta' [graus]
				aux.s = current_s; // aquells punt tindran associada lamateixa estacio
			
				traj_points_aux.push_back(aux);
				feasible_traj_points_aux.push_back(1);
			}
			
			traj_points.push_back(traj_points_aux);	//afegim el vecto de elements pertanyents a la mateixa 'staion'
			traj_points_aux.clear();
			
			feasible_traj_points.push_back(feasible_traj_points_aux); // la matriu sera tota de 1s (ja que no hi han obstacles)
			feasible_traj_points_aux.clear();
		}
		max_feasible_station = desired_stations.back();	// la station final es la ultima
	}
	// Mostreig quan hi han obstacle a la discretitzacio espaial feta 
	else // si hi ha un obstacle a l'espai discretitzat, acabem en l'station' asociada a l'obstacle
	{
		vector <float> used_stations;
		for(int i=0; i<(int)desired_stations.size(); i++) // calculem les stations on hi han obstacles
		{
			float current_station=desired_stations[i];
			if (current_station<obs_station) // aquest 'obs_station' es la 'station amb obstacle mes allunyada
			{
				used_stations.push_back(current_station);
			}
			else
			{
				break;
			}
		}
		used_stations.push_back(obs_station);
		
		//Elements del output
		//vector <vector<int> > feasible_traj_points; // matriu d'elements de 1s o 0s que representa si la posocio amb un offset i station determinada esta ocuoda (0) o no per un obstacle (1) 
		//vector <vector<C_SamplePoint> > traj_points; // matriu d'elements C_SamplePoint que representa la dsicretitzacio de tota l'area d'actuacio 
		
				
		for (int i=0; i<(int)used_stations.size(); i++) // primer omplim feasible_traj_points amb 0s i traj_points amb elements C_SamplePoint pero de tot 0s
		{
			vector <int> feasible_traj_points_aux;
			vector <C_SamplePoint> traj_points_aux;
			C_SamplePoint aux2;
			
			for (int j=0; i<(int)used_offsets.size(); j++)
			{
				feasible_traj_points_aux.push_back(0); 
				traj_points_aux.push_back(aux2);
			}
			
			feasible_traj_points.push_back(feasible_traj_points_aux); // tot 0s (com si cap posicio estigues disponible, perque hi ha obstacle)
			feasible_traj_points_aux.clear();
			
			traj_points.push_back(traj_points_aux); // Tots elements de C_SamplePoint els de per defecte (0,0,0,0,-1)
			traj_points_aux.clear();
		}
			
		float current_s;
		float head_traj;
		
		for(int i=0; i<(int)used_stations.size(); i++)
		{
			current_s=used_stations[i];
			head_traj=constrain_angle( atan2( (center_lane[1][current_s+1] - center_lane[1][current_s]) , (center_lane[0][current_s+1] - center_lane[0][current_s])  )*(180/M_PI) , 360.0 ); //calcul orientacio
			
			float current_offset;
			
			for (int j=0; j<(int)used_offsets.size(); j++) // PER A CADA OFFSET (FILA)
			{
				current_offset=used_offsets[j];
				
				//construim cada un dels elements de la matriu 'traj_points' --> elements de la clase C_SamplePoint
				C_SamplePoint aux; // End point de la 'station' asociada al obstacle
				aux.x = center_lane[0][current_s]+offsets[current_offset]*cos((head_traj*M_PI/180.0) + M_PI/2); // coord X del obstacle
				aux.y = center_lane[1][current_s]+offsets[current_offset]*sin((head_traj*M_PI/180.0) + M_PI/2); // coord Y del obstacle
				aux.k = 1 / ( (1/center_lane[2][current_s])-offsets[current_offset] );
				aux.head = head_traj; // 'theta' [graus]
				aux.s = current_s; // aquells punt tindran associada la mateixa estacio
			
				
				traj_points[i][current_offset]=aux;//traj_points[current_offset][i]=aux;
				feasible_traj_points[i][current_offset]=1;
			}
		}
		max_feasible_station = obs_station;
	}
}


void C_Sampling::sample_ahead (float initial_point_GP_path_i, float desired_station, float r_col, vector <vector<C_SamplePoint> > &traj_points, vector <vector<int> > &feasible_traj_points, float &max_feasible_station, int &lane_blocked)
{
	int flag_obstacle_inside=0;
	lane_blocked=0;
	
	if (desired_station > final_station) //% For preventing going out of bounds in our center_lane
	{
		desired_station=final_station;
	}
	
	float max_desired_GP_path_i = initial_point_GP_path_i + desired_station;
	
	
	vector <C_Obstacle> closest_obs; // vector dels objectes de la clase C_Obstacle dins el interval de 'indexos' analitzat
	vector <float> obs_stations; // vector que indica les 'stations' que més aprop es troben als respectius obstacles
	int no_obstacle; // valdra 0 si no hi ha cap obstacle
	
	static_obs.get_all_closest_obs_interval( initial_point_GP_path_i, max_desired_GP_path_i, closest_obs, obs_stations, no_obstacle); // retorna un vector de tots els obstacles visibles i ordenats de mes aprop a més lluny i dins un interval de 'stations' indicat pel valors de 'station' major i el valor de 'station' menor tambe retorna el vector associat a les estacions d'aquests objectes (tambe ordenat) i una variable per indicar quan no hi ha cap objecte visible 

	int num_obs=closest_obs.size();
	float obs_station;
	float obs_dist;
	
	vector <float> used_offsets; //vector on s'emmagazemaran els indexs dels offset del vector de ofsets que es poden emprar (perque no tenen un obstacle que ho impedeixi)
	vector <C_SamplePoint> end_points; // vector on s'emmagatzemen els punts offset accesibles d'una 'station'
	float head_traj; //orientacio 

	if (num_obs>0)
	{
		for (int o=0; o<num_obs; o++) // PER CADA OBSTACLE
		{			
			
			Search_closest_point(closest_obs[o].pos[0], closest_obs[o].pos[1], center_lane[0], center_lane[1], 0, obs_station, obs_dist); 
			
			if (obs_station>desired_station)
			{
				obs_station = desired_station; // com a molt es relacionara amb la ultima 'station'
			}
			
			int num_obs_same_station; // numero de obstacles en la mateixa estacio
			if (o <= num_obs-2)
			{
				if ( obs_stations[o] == obs_stations[o+1] ) // el obstacle actual i el proxim es troben a la mateixa estacio
				{
					num_obs_same_station=2;
				}
				else
				{
					num_obs_same_station=1;
				}
			}
			else 
			{
				num_obs_same_station=1;
			}
			
			flag_obstacle_inside=1;
			//used_offsets.clear();
			//end_points.clear();
			
			head_traj= constrain_angle( atan2( (center_lane[1][obs_station+1] - center_lane[1][obs_station]) , (center_lane[0][obs_station+1] - center_lane[0][obs_station])  )*(180/M_PI) , 360.0 ); //calcul orientacio

			for (int j=0; j< (int) offsets.size(); j++) // PER CADA OFFSET
			{
				C_SamplePoint aux; // End point de la 'station' asociada al obstacle
				aux.x = center_lane[0][obs_station]+offsets[j]*cos((head_traj*M_PI/180.0) + M_PI/2); // coord X del obstacle
				aux.y = center_lane[1][obs_station]+offsets[j]*sin((head_traj*M_PI/180.0) + M_PI/2); // coord Y del obstacle
				aux.k = 1 / ( (1/center_lane[2][obs_station])-offsets[j] );
				aux.head = head_traj; // 'theta' [graus]
				aux.s = obs_station; // aquells punt tindran associada lamateixa estacio
			
				end_points.push_back(aux);
								
				if (num_obs_same_station==1)
				{
					if ( check_col_sample_point(aux, r_col, closest_obs[o]) == 0 ) // cas en que en aquella posicio no colisiona obstacle amb cotxe
					{
						used_offsets.push_back(j);
					}
				}
				else if (num_obs_same_station==2)
				{
					if ( (check_col_sample_point(aux, r_col, closest_obs[o]) == 0) && (check_col_sample_point(aux, r_col, closest_obs[o+1]) == 0) )
					{
						used_offsets.push_back(j);
					}
				}
				else
				{
				   // NO poden haver mes de dos obstacle mateixa 'station' !	
				}
			}
			
			if (used_offsets.empty()==1) // no hi ha cap offset que no estigui obstruit per un obstacle (cap index del vector de offsets disponible)
			{
				lane_blocked=1;
				used_offsets.push_back(ceil(offsets.size()/2)-1); 
			}
			else
			{
				if (used_offsets.size() == offsets.size()) // cap offset es pbstruit per l'obstacle
				{
					flag_obstacle_inside=0;  // el obstacle es pot menystenir, ja que no obstaculitza l'acces a cap offset
				}
			}
			
			if (flag_obstacle_inside==1)
			{
				break; // acabem aqui, ja em detectat un obstacle i aquest es el que considerem que crea la obstruccio
			}
		}
	}
	
		
	if (flag_obstacle_inside == 0) // % If there is NO obstacle in all the sampling area
	{
		//Elements del output
		//vector <vector<int> > feasible_traj_points; // matriu d'elements de 1s o 0s que representa si la posocio amb un offset i station determinada esta ocuoda (0) o no per un obstacle (1) 
		//vector <vector<C_SamplePoint> > traj_points; // matriu d'elements C_SamplePoint que representa la dsicretitzacio de tota l'area d'actuacio 
		
		//float current_s;
		//float head_traj
		
		head_traj=constrain_angle( atan2( (center_lane[1][desired_station+1] - center_lane[1][desired_station]) , (center_lane[0][desired_station+1] - center_lane[0][desired_station])  )*(180/M_PI) , 360.0 ); //calcul orientacio
			
		vector <C_SamplePoint> traj_points_aux; 
		vector <int> feasible_traj_points_aux;
			
		for (int j=0; j<(int)offsets.size(); j++) // PER A CADA OFFSET (FILA)
		{
				//construim cada un dels elements de la matriu 'traj_points' --> elements de la clase C_SamplePoint
			C_SamplePoint aux; // End point de la 'station' asociada al obstacle
			aux.x = center_lane[0][desired_station]+offsets[j]*cos((head_traj*M_PI/180.0) + M_PI/2); // coord X del obstacle
			aux.y = center_lane[1][desired_station]+offsets[j]*sin((head_traj*M_PI/180.0) + M_PI/2); // coord Y del obstacle
			aux.k = 1 / ( (1/center_lane[2][desired_station])-offsets[j] );
			aux.head = head_traj; // 'theta' [graus]
			aux.s = desired_station; // aquells punt tindran associada lamateixa estacio
		
			traj_points_aux.push_back(aux);
			feasible_traj_points_aux.push_back(1);
		}
			
		traj_points.push_back(traj_points_aux);	//afegim el vecto de elements pertanyents a la mateixa 'staion' (en aquest cas nomes tindra una columna)
		traj_points_aux.clear();
			
		feasible_traj_points.push_back(feasible_traj_points_aux); // la matriu sera tota de 1s (ja que no hi han obstacles)
		feasible_traj_points_aux.clear();
		
		max_feasible_station = desired_station;		
		
	}
	else // % If there is an obstacle inside the sampling area, we end at its station
	{
		vector <C_SamplePoint> traj_points_aux;
		vector <int> feasible_traj_points_aux;
		C_SamplePoint aux; 
		
		// posem vector 'feasible_traj_points' tot a 0
		for (int j=0; j<(int)used_offsets.size(); j++)
		{
			feasible_traj_points_aux.push_back(0);
			traj_points_aux.push_back(aux);
		}
		
		feasible_traj_points.push_back(feasible_traj_points_aux);
		traj_points.push_back(traj_points_aux);
		
		float current_offset;
		
		for (int j=0; j<(int)used_offsets.size(); j++) 
		{
			current_offset=used_offsets[j];
			
			traj_points[0][current_offset]=end_points[current_offset]; // afegeixo element de la classe C_SamplePoint que no es el de per defecte
			feasible_traj_points[0][current_offset]=1; // afegeixo un 1 (punt de discretitzacio accesible) nomes te una 'station'
		}
		
		max_feasible_station=obs_station;
	}
}


void C_Sampling::sample_ahead_other_lane (float initial_point_GP_path_i, float desired_station, float r_col, vector <vector<float> > other_center_lane, vector <vector<C_SamplePoint> > &traj_points, vector <vector<int> > &feasible_traj_points, float &max_feasible_station, int &lane_blocked)
{
	int flag_obstacle_inside=0;
	lane_blocked=0; //% Flag to indicate if the lane is totally blocked by an obstacle
	
	if (desired_station > final_station)
	{
		desired_station=final_station;
	}
	
	float max_desired_GP_path_i = initial_point_GP_path_i + desired_station;
	
	vector <C_Obstacle> closest_obs; // vector dels objectes de la clase C_Obstacle dins el interval de 'indexos' analitzat
	vector <float> obs_stations; // vector que indica les 'stations' que més aprop es troben als respectius obstacles
	int no_obstacle; // valdra 0 si no hi ha cap obstacle
	
	static_obs.get_all_closest_obs_interval( initial_point_GP_path_i, max_desired_GP_path_i, closest_obs, obs_stations, no_obstacle); // retorna un vector de tots els obstacles visibles i ordenats de mes aprop a més lluny i dins un interval de 'stations' indicat pel valors de 'station' major i el valor de 'station' menor tambe retorna el vector associat a les estacions d'aquests objectes (tambe ordenat) i una variable per indicar quan no hi ha cap objecte visible 

	int num_obs=closest_obs.size();
	float obs_station;
	float obs_dist;
	
	vector <float> used_offsets; //vector on s'emmagazemaran els indexs dels offset del vector de ofsets que es poden emprar (perque no tenen un obstacle que ho impedeixi)
	vector <C_SamplePoint> end_points; // vector on s'emmagatzemen els punts offset accesibles d'una 'station'
	float head_traj; //orientacio 

	if (num_obs>0)
	{
		for (int o=0; o<num_obs; o++) // PER CADA OBSTACLE
		{			
			Search_closest_point(closest_obs[o].pos[0], closest_obs[o].pos[1], other_center_lane[0], other_center_lane[1], 0, obs_station, obs_dist); 
			
			int num_obs_same_station; // numero de obstacles en la mateixa estacio
			
			if (o <= num_obs-2)
			{
				if ( obs_stations[o] == obs_stations[o+1] ) // el obstacle actual i el proxim es troben a la mateixa estacio
				{
					num_obs_same_station=2;
				}
				else
				{
					num_obs_same_station=1;
				}
			}
			else 
			{
				num_obs_same_station=1;
			}
			
			flag_obstacle_inside=1;
			//used_offsets.clear();
			//end_points.clear();
			
			head_traj= constrain_angle( atan2( (other_center_lane[1][obs_station+1] - other_center_lane[1][obs_station]) , (other_center_lane[0][obs_station+1] - other_center_lane[0][obs_station])  )*(180/M_PI) , 360.0 ); //calcul orientacio

			for (int j=0; j< (int) offsets.size(); j++) // PER CADA OFFSET
			{
				C_SamplePoint aux; // End point de la 'station' asociada al obstacle
				aux.x = other_center_lane[0][obs_station]+offsets[j]*cos((head_traj*M_PI/180.0) + M_PI/2); // coord X del obstacle
				aux.y = other_center_lane[1][obs_station]+offsets[j]*sin((head_traj*M_PI/180.0) + M_PI/2); // coord Y del obstacle
				aux.k = 1 / ( (1/other_center_lane[2][obs_station])-offsets[j] );
				aux.head = head_traj; // 'theta' [graus]
				aux.s = obs_station; // aquells punt tindran associada lamateixa estacio
			
				end_points.push_back(aux);
								
				if (num_obs_same_station==1)
				{
					if ( check_col_sample_point(aux, r_col, closest_obs[o]) == 0 ) // cas en que en aquella posicio no colisiona obstacle amb cotxe
					{
						used_offsets.push_back(j);
					}
				}
				else if (num_obs_same_station==2)
				{
					if ( (check_col_sample_point(aux, r_col, closest_obs[o]) == 0) && (check_col_sample_point(aux, r_col, closest_obs[o+1]) == 0) )
					{
						used_offsets.push_back(j);
					}
				}
				else
				{
				   // NO poden haver mes de dos obstacle mateixa 'station' !	
				}
			}
			
			if (used_offsets.empty()==1) // no hi ha cap offset que no estigui obstruit per un obstacle (cap index del vector de offsets disponible)
			{
				lane_blocked=1;
				used_offsets.push_back(ceil(offsets.size()/2)-1); 
			}
			else
			{
				if (used_offsets.size() == offsets.size()) // cap offset es pbstruit per l'obstacle
				{
					flag_obstacle_inside=0;  // el obstacle es pot menystenir, ja que no obstaculitza l'acces a cap offset
				}
			}
			
			if (flag_obstacle_inside==1)
			{
				break; // acabem aqui, ja em detectat un obstacle i aquest es el que considerem que crea la obstruccio
			}
		}
	}
	
	if (flag_obstacle_inside == 0) // % If there is NO obstacle in all the sampling area
	{
		//Elements del output
		//vector <vector<int> > feasible_traj_points; // matriu d'elements de 1s o 0s que representa si la posocio amb un offset i station determinada esta ocuoda (0) o no per un obstacle (1) 
		//vector <vector<C_SamplePoint> > traj_points; // matriu d'elements C_SamplePoint que representa la dsicretitzacio de tota l'area d'actuacio 
		
		//float current_s;
		//float head_traj
		head_traj=constrain_angle( atan2( (center_lane[1][desired_station+1] - center_lane[1][desired_station]) , (center_lane[0][desired_station+1] - center_lane[0][desired_station])  )*(180/M_PI) , 360.0 ); //calcul orientacio
			
		vector <C_SamplePoint> traj_points_aux; 
		vector <int> feasible_traj_points_aux;
			
		for (int j=0; j<(int)offsets.size(); j++) // PER A CADA OFFSET (FILA)
		{
				//construim cada un dels elements de la matriu 'traj_points' --> elements de la clase C_SamplePoint
			C_SamplePoint aux; // End point de la 'station' asociada al obstacle
			aux.x = other_center_lane[0][desired_station]+offsets[j]*cos((head_traj*M_PI/180.0) + M_PI/2); // coord X del obstacle
			aux.y = other_center_lane[1][desired_station]+offsets[j]*sin((head_traj*M_PI/180.0) + M_PI/2); // coord Y del obstacle
			aux.k = 1 / ( (1/other_center_lane[2][desired_station])-offsets[j] );
			aux.head = head_traj; // 'theta' [graus]
			aux.s = desired_station; // aquells punt tindran associada lamateixa estacio
		
			traj_points_aux.push_back(aux);
			feasible_traj_points_aux.push_back(1);
		}
			
		traj_points.push_back(traj_points_aux);	//afegim el vecto de elements pertanyents a la mateixa 'sation' (en aquest cas nomes tindra una columna)
		traj_points_aux.clear();
			
		feasible_traj_points.push_back(feasible_traj_points_aux); // la matriu sera tota de 1s (ja que no hi han obstacles)
		feasible_traj_points_aux.clear();
		
		max_feasible_station = desired_station;		
	}
	else // % If there is an obstacle inside the sampling area, we end at its station
	{
		vector <C_SamplePoint> traj_points_aux;
		vector <int> feasible_traj_points_aux;
		C_SamplePoint aux; 
		
		// posem vector 'feasible_traj_points' tot a 0
		for (int j=0; j<(int)used_offsets.size(); j++)
		{
			feasible_traj_points_aux.push_back(0);
			traj_points_aux.push_back(aux);
		}
		
		feasible_traj_points.push_back(feasible_traj_points_aux);
		traj_points.push_back(traj_points_aux);
		
		float current_offset;
		
		for (int j=0; j<(int)used_offsets.size(); j++) 
		{
			current_offset=used_offsets[j];
			
			traj_points[0][current_offset]=end_points[current_offset]; // afegeixo element de la classe C_SamplePoint que no es el de per defecte
			feasible_traj_points[0][current_offset]=1; // afegeixo un 1 (punt de discretitzacio accesible) nomes te una 'station'
		}
		
		max_feasible_station=obs_station;
	}
}


void C_Sampling::sample_lane (float r_col, S_lane_sampling& lane_sampling) 
{
	float ini_point_GP_path_i = initial_point_GP_path_i;
	vector <float > desired_stations;
	desired_stations=stations;
	
	vector <vector<C_SamplePoint> > traj_points_new; //matriu d'elements C_SamplePoint que representa la discretitzacio espaial del tram de carril
	vector <vector<int> > feasible_traj_points_new; //matriu amb 1s en les posicion de la discretitzacio espaial accesibles (no obstaculitzades per cap obstacle)
	float new_feasible_station; //'station' accesible mes allunyada del tram de carril
	int new_lane_blocked; // indica amb un 1 si carril totalment bloquejat
	
	sample (initial_point_GP_path_i, desired_stations, r_col, traj_points_new, feasible_traj_points_new, new_feasible_station, new_lane_blocked); // funcio per discretitzar tram de carril i cpnsiderant obstacles
	
	vector <vector<C_SamplePoint> > traj_points; // matriu que representa tots els offsets de les estacions (discretitzacions longitudinals) del carril amb els seus punts offsets 
	traj_points=traj_points_new;
	vector <vector<int> > feasible_traj_points; // matriu amb 1s amb els index de les posicions accesibles per la discretitzacio
	feasible_traj_points=feasible_traj_points_new;
	float current_feasible_station;// 'sation' actual mes allunyada que ens trobem
	current_feasible_station=new_feasible_station;
	int lane_blocked; // indica amb 1, si el carril esta completament bloquejat
	lane_blocked=new_lane_blocked;
	
	vector <int > obstruction_here;	//creacio de vector que indica amb 1s en quines 'stations' hi ha un obstacle que pertorba algun punt de mostreig (almenys 1)
	
	for (int i=0; i<(int)feasible_traj_points[0].size(); i++)
	{
		obstruction_here.push_back(0); // creem vecor de 0s de longitud igual al numero de 'stations'
	}
	
	if ( sum_int(feasible_traj_points.back())< (int) offsets.size() ) // es cumpleix si hi ha algun 0 
	{
		obstruction_here.back()=1; // posem un 1 a la ultima 'station'
	}
	
	//Un cop hem emprat el vector 'dists' per fer una primera discretitzacio del carril (la del tram mes proper) 
	// faig una altre discretitzaci fins al punt final determinat per la 'lookahead_dist' --> abast del nostre sensor
	// aquesta segona discretitzacio es mes basta --> s'incloura una 'station' al final i unes altres entre la 1a discretitzacio i la final
	// aquestes altres sempre seran augmentant la ultima el valor de la 'station'
	
	
	float first_sampling_index= obstruction_here.size()-1; // $ Station index where the first sampling ends // sera igual al numero de 'station'-1
	
	float increment_station_ahead= stations.back()+1; // % Desired station increment ahead (index de punt al qual voldrem arrivar a la proxima iteracio) -->> +1 ja que per nosaltre la 1a station es la '0'

	//Recorregut de les 'stations' 
	float desired_station_ahead;
	while ( (current_feasible_station<final_station) && (lane_blocked==0) ) // mentre que no hagim arrivat a la ultima 'station' del tram de carretera i no ens trobem la carretera totalment bloquejada
	{
		
		if ( (current_feasible_station+increment_station_ahead)>final_station )
		{
			desired_station_ahead=final_station;
		}
		else
		{
			desired_station_ahead = current_feasible_station + increment_station_ahead;
		}
		
		ini_point_GP_path_i = initial_point_GP_path_i + current_feasible_station;
		
		vector <vector<C_SamplePoint> > traj_points_new;
		vector <vector<int> > feasible_traj_points_new;
		float new_feasible_station;
		int new_lane_blocked;
		
		// muestreo de la estacion perteneciencient al maximo alcance del sensor --> lookahead_dist ( se añadiran al resto de stations)
		sample_ahead (ini_point_GP_path_i, desired_station_ahead, r_col, traj_points_new, feasible_traj_points_new, new_feasible_station, new_lane_blocked);
		
		
		/* prueba
		for (int i=0; i<traj_points_new.size(); i++)
		{
			cout << traj_points_new[i][0].s << endl ;
		}
		*/
		
		
		// añadimos las nuevas 'stations' a las 'stations' determinadas por la distancia de muestreo
		for (int i=0; i<(int)traj_points_new.size(); i++)
		{
			traj_points.push_back(traj_points_new[i]);
		}
		
		for (int i=0; i<(int)feasible_traj_points_new.size(); i++)
		{
			feasible_traj_points.push_back(feasible_traj_points_new[i]);
		}
		
		current_feasible_station=new_feasible_station;
		lane_blocked=new_lane_blocked;
		
		if ( sum_int(feasible_traj_points.back()) < (int)offsets.size() ) // hi ha algun offset d'aquella estacio amb valor 0
		{
			obstruction_here.push_back(1); // indiquem que hi ha obstruccio
		}
		else
		{
			obstruction_here.push_back(0);	// cas no hi ha obstruccio
		}
	}
	
	float max_feasible_station = current_feasible_station;
	
	vector <float > traj_stations; // calcul del vector amb les 'stations' del mostreig del tram del carril

	for (int i=0; i< (int) obstruction_here.size(); i++)
	{
		traj_stations.push_back(0);
	}
		
	float index=0;
	int cont=0;
	for (int s=0; s<(int)traj_stations.size(); s++)  // recorrem les 'staions'
	{
		cont=0;
		index=0;
		for (int i=0; i< (int) feasible_traj_points[s].size(); i++) 
		{
			if ( (feasible_traj_points[s][i]==1) && (cont==0) )
			{
				index=i;
				cont=1;
			}
		}
		traj_stations[s]=traj_points[s][index].s;
	}
	
	// construim el 'output' --> estructura del tipus ''S_lane_sampling'
	lane_sampling.traj_points = traj_points;
    lane_sampling.feasible_traj_points = feasible_traj_points;
    lane_sampling.traj_stations = traj_stations;
    lane_sampling.obstruction_here = obstruction_here;
    lane_sampling.max_feasible_station = max_feasible_station;
    lane_sampling.lane_blocked = lane_blocked;
    lane_sampling.first_sampling_index = first_sampling_index;
                 
}





void C_Sampling::traj_combinations(S_lane_sampling lane_sampling, S_lane_sampling_comb& new_lane_sampling )
{
	
	// treiem els membre de la estructua 'lane_sampling'
	vector <vector<C_SamplePoint> > traj_points; // matriu amb la discretitzacio espaial i la informacio de cadascun dels punts en que ha estat feta la discretització
	vector <vector<int> > feasible_traj_points; // matriu de 1s i/o 0s --> 1 indica que no hi ha cap obstacle pertorbant aquell punt, 0 es que hi ha obstacle
	vector<float> traj_stations; // numeros de punts (indexos del carril) on es troba cada una de les 'stations' amb que discretitzem el carril ( cada columna de 'traj_points' compartira la mateixa 'station' )
	vector<int> obstruction_here; // indica amb 0 la 'station' on no hi ha cap obstacle, i amb un 1 on hi ha obstacle (encara que sigui nomes en un dels seus punts de mostreig(offset))
	float max_feasible_station; // valor del index de la 'station' mes allunyada
	int lane_blocked; // indica amb un 1 que no es pot passar pel carril (totalment bloquejat) i amb un 0 de que si es pot pasar ( tot i que por tenir obstacles)
	float first_sampling_index; // index 'numero de punt' que primer es mostreja del carril
	
			
	traj_points = lane_sampling.traj_points;
    feasible_traj_points = lane_sampling.feasible_traj_points;
    traj_stations = lane_sampling.traj_stations;
    obstruction_here = lane_sampling.obstruction_here;
    max_feasible_station = lane_sampling.max_feasible_station;
    lane_blocked = lane_sampling.lane_blocked;
    first_sampling_index = lane_sampling.first_sampling_index;
	
	int num_stations=traj_stations.size();
	
	vector<float> obstruction_indexes;	// fem un vector amb tots els indexs de les 'stations' del carril on hi ha obstacle --> ([ 0 , 4] voldra dir que la 1a 'station' i la 5a tenen obstacle
	vector<float> sorted_indexes; // ordenem de mes allunyat a mes proper
	for (int i=0; i< (int) obstruction_here.size(); i++)
	{
		if (obstruction_here[i]==1)
		{
			obstruction_indexes.push_back(i);
		}
	}
	
	float last_obstruction_index;
	
	if (obstruction_indexes.empty()==1) // cas no hi ha cap obstacle en cap estacio
	{
		// no fem res
	}
	else // cas en que si hi han obstacles
	{
		sorted_indexes=obstruction_indexes;
		sort(sorted_indexes.begin(), sorted_indexes.end()); // ordenem indexs
		reverse(sorted_indexes.begin(), sorted_indexes.end()); // ordenem de forma decreixent
		
		last_obstruction_index = sorted_indexes[0]; // agafem el index de la ultima 'station' que te associada un obstacle
		int num_obstructions=sorted_indexes.size(); // numero de 'stations' que tenen un obstacle associat
		
		
		// de la matriu 'feasible_traj_points' --> Posem totes les columnes de offsets de les 'stations' que estan mes allunyades de la ultima station amb obstacle, igual que la columnna de offsets d'aquesta ultima 'station amb obstacle
		for (int i=last_obstruction_index+1; i<=(int)num_stations-1; i=i+1) // posem totes les station per darrere igual que la de la 'station' amb l'obstacle ??¿?¿ --> % Cleaning of the last stations starting in the last obstruction index
		{
			feasible_traj_points[i] = feasible_traj_points[last_obstruction_index];						
		}
		
		float index;
		for (int i=0; i< (int) num_obstructions; i++)
		{
			index = sorted_indexes[i] - 1; //% Index of the left station of each obstruction, starting from the end (index de la 'station' que queda a la esquerra de cada obstruccio)
			
			if (index <= first_sampling_index) // 
			{
				break;
			}
			
			while (1)
			{
				if (index <= first_sampling_index)
				{
					break;
				}
				
				if (obstruction_here[index] == 0) // no hi ha obstruccio en la 'station'
				{
					feasible_traj_points[index] = feasible_traj_points[sorted_indexes[i]];
				}
				else
				{
					break;
				}
				
				index = index - 1;
			}
		}
	}	
	
	// % Once we have recomputed the feasible_traj_points matrix, we can compute what stations need to follow the same previous 
    // % offset and what stations need to have all offsets combinations
    
    vector<float> offset_combinations; // vector que indica quines 'stations' han de seguir el mateix offset previ i quines 'stations' poden seguir qualsevol offset
    for (int i=0; i< (int) num_stations; i++)
	{
		offset_combinations.push_back(0);
	}
	
	int last_offset_comb=0;	
	 
	if (obstruction_indexes.empty()==1) // % There are no obstructions
	{
		// % No need to do anything, because is all zeros already
	}
	else // % There are obstructions
	{
		for (int i=first_sampling_index + 1; i< (int) num_stations; i++) // recorrem desde la seguent 'station' amb obstacle fins a totes
		{
			if (i>last_obstruction_index)
			{
				break;
			}
			
			if (obstruction_here[i]==1) // hi ha un obstacle associat a aquesta 'station'
			{
				if (last_offset_comb == 0) // habia de seguir un offset prefixat
				{
					offset_combinations[i] = 1; // ha de seguir tots els offsets disponibles
				}
				else // no habia de seguir offset prefixat
				{
					last_offset_comb = 0;
				}
			}
			else
			{
				if ( ( sum_int( feasible_traj_points[i] ) < (int) offsets.size() ) && (last_offset_comb == 0) )
				{
					offset_combinations[i] = 1;
                    last_offset_comb = 1;
					
				}
			}
		}
	}
	
	
	// construim el 'output' --> estructura del tipus 'S_lane_sampling_comb'
	new_lane_sampling.traj_points = traj_points;
    new_lane_sampling.feasible_traj_points = feasible_traj_points;
    new_lane_sampling.traj_stations = traj_stations;
    new_lane_sampling.obstruction_here = obstruction_here;
    new_lane_sampling.max_feasible_station = max_feasible_station;
    new_lane_sampling.lane_blocked = lane_blocked;
    new_lane_sampling.first_sampling_index = first_sampling_index;
    new_lane_sampling.offset_combinations = offset_combinations;
	
	
}



void C_Sampling::avoidance_other_lane (float r_col, vector <vector<float> > other_center_lane, float collision_index, S_lane_sampling &lane_sampling, int &lane_change_finished)
{
	float ini_point_GP_path_i = initial_point_GP_path_i;
	
	vector<float> indexs; //indica quines stations es troben despres del 'index de colisio'
	
	for (int i=0; i<(int)stations.size(); i++) // recorrem desde la seguent 'station' amb obstacle fins a totes
	{
		if (stations[i]<=collision_index) // % Forcing that the first sampling cannot overpass the avoidance station
		{
			indexs.push_back(1);			
		}
		else
		{
			indexs.push_back(0);
		}
	}
	
	vector<float> desired_stations; //stations d'abans de la colisio
	
	for (int i=0; i<(int)stations.size(); i++) // recorrem desde la seguent 'station' amb obstacle fins a totes
	{
		if (indexs[i]==1)
		{
			desired_stations.push_back(stations[i]);
		}
	}
		
		
	vector <vector<C_SamplePoint> > traj_points_new;
	vector <vector<int> > feasible_traj_points_new;
	float new_feasible_station;
	int new_lane_blocked;
	
	sample ( ini_point_GP_path_i, desired_stations, r_col, traj_points_new, feasible_traj_points_new, new_feasible_station, new_lane_blocked); // funcio per mostrejar

	vector <vector<C_SamplePoint> > traj_points=traj_points_new;
	vector <vector<int> > feasible_traj_points=feasible_traj_points_new;
	float current_feasible_station=new_feasible_station;
	int lane_blocked=new_lane_blocked;
	
	vector<int> obstruction_here;
	
	//inicialitzem tot a 0 --> no hi ha obstruction
	for (int i=0; i<(int)feasible_traj_points.size(); i++) 
	{
		obstruction_here.push_back(0);
	}
	
	if ( sum_int(feasible_traj_points.back()) < (int)offsets.size() ) // cas en que algun offset o mes de un, de la ultima 'station', te una obstruccio
	{
		obstruction_here.back() = 1; // posem el index de aquella ultima 'station' del vector 'obstruction_here' a 1
	}
	
	float first_sampling_index= obstruction_here.size()-1; // % Station index where the first sampling ends
	float increment_station_ahead= stations.back(); // % Desired station increment ahead
	
	lane_change_finished=0; // encara no ha acabat
	
	float desired_station_ahead;
	
	traj_points_new.clear();
	feasible_traj_points_new.clear();
		
	
	if (collision_index<final_station)
	{
		while ( (current_feasible_station<final_station) && (lane_blocked==0) ) // PER A CADA STATION % We stop when arriving to the final station, or encountering a blocking
		{
			//% Iteration in the station coordinate:
			if ( (current_feasible_station<collision_index) && ( (current_feasible_station+increment_station_ahead)>collision_index ) )
			{
				desired_station_ahead = collision_index; //% Is relative!				
			}
			else
			{
				desired_station_ahead = current_feasible_station + increment_station_ahead; //% Is relative!
			}
			
			if ( ((current_feasible_station + increment_station_ahead) > final_station) && (desired_station_ahead =! collision_index) )
			{
				desired_station_ahead = final_station; //% Is relative!
			}
			
			ini_point_GP_path_i = initial_point_GP_path_i + current_feasible_station; //% Is absolute!
			
			
			
			
			if (current_feasible_station < collision_index)
			{
				sample_ahead (ini_point_GP_path_i, desired_station_ahead, r_col, traj_points_new, feasible_traj_points_new, new_feasible_station, new_lane_blocked);
			}
			else // % collision_index or ahead
			{
				sample_ahead_other_lane (ini_point_GP_path_i, desired_station_ahead, r_col, other_center_lane, traj_points_new, feasible_traj_points_new, new_feasible_station, new_lane_blocked);
				
				if (new_lane_blocked==0)
				{
					if ( (desired_station_ahead-current_feasible_station) >= increment_station_ahead )
					{
						lane_change_finished = 1;
					}
				}
				else
				{
					desired_station_ahead = new_feasible_station;
							
					//vector <vector<C_SamplePoint> >	traj_points_new;				
					//vector <vector<int> > feasible_traj_points_new;
					//float new_feasible_station;
					//int new_lane_blocked;
					
					sample_ahead ( ini_point_GP_path_i, desired_station_ahead, r_col, traj_points_new, feasible_traj_points_new, new_feasible_station, new_lane_blocked);
					
				}
			}
			
			for (int i=0; i<(int)traj_points_new.size(); i++) 
			{
				traj_points.push_back(traj_points_new[i]);
				feasible_traj_points.push_back(feasible_traj_points_new[i]);
			}
			
			current_feasible_station = new_feasible_station; //% Is relative!
            lane_blocked = new_lane_blocked;
            
            if ( sum_int(feasible_traj_points.back())<(int)(offsets.size()) )
            {
				obstruction_here.push_back(1);
				
			}
			else
			{
				obstruction_here.push_back(0);
			}
		
		}
		
	}
	
	float max_feasible_station = current_feasible_station; //% Is relative! 
	
	vector<float> traj_stations;
	
	for (int i=0; i<(int)obstruction_here.size(); i++) 
	{
		traj_stations.push_back(0);
	}
	
	float index;
	for (int s=0; s<(int)traj_stations.size(); s++) 
	{
		index=0;
		for (int j=0; j<(int)feasible_traj_points[s].size(); j++) 
		{
			if ( feasible_traj_points[s][j]==0 )
			{
				index=j;
				break;
			}
				
		}
		traj_stations[s]=traj_points[s][index].s;
	}
	
	lane_sampling.traj_points = traj_points;
    lane_sampling.feasible_traj_points = feasible_traj_points;
    lane_sampling.traj_stations = traj_stations;
    lane_sampling.obstruction_here = obstruction_here;
    lane_sampling.max_feasible_station = max_feasible_station;
    lane_sampling.lane_blocked = lane_blocked;
    lane_sampling.first_sampling_index = first_sampling_index;
	
	
} 

