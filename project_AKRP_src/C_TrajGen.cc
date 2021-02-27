#include "C_TrajGen.h" 

using namespace std; 

//constructor 
C_TrajGen::C_TrajGen ( S_lane_sampling_comb lane_sampling1, C_SamplePoint initial_point1, float initial_point_GP_path_i1, float resolution1, C_StaticObs static_obs1)
{	
	// extraiem elements de la struct S_lanesampling_comb
	traj_points = lane_sampling1.traj_points;
    feasible_traj_points = lane_sampling1.feasible_traj_points;
	traj_stations = lane_sampling1.traj_stations;
    obstruction_here = lane_sampling1.obstruction_here;
    max_feasible_station = lane_sampling1.max_feasible_station;
    lane_blocked = lane_sampling1.lane_blocked;
    first_sampling_index = lane_sampling1.first_sampling_index;
    offset_combinations = lane_sampling1.offset_combinations;
    
    num_stations = traj_stations.size();
    num_offsets = feasible_traj_points[0].size(); // per exemple els de la 1a 'station' (sempre sera el mateix per cada 'station')
            
    initial_point = initial_point1;
    initial_point_GP_path_i = initial_point_GP_path_i1;
    resolution = resolution1;
    static_obs = static_obs1;
    		
}

//default constructor 

C_TrajGen::C_TrajGen ()
{
	C_SamplePoint aux;
	vector<C_SamplePoint> traj_aux;
	traj_aux.push_back(aux);
	vector <vector<C_SamplePoint> > traj_aux_aux;
	traj_aux_aux.push_back(traj_aux);
	traj_points=traj_aux_aux;
	
	vector <int > feasible_traj_points_aux;
	feasible_traj_points_aux.push_back(0);
	vector <vector<int> > feasible_traj_points_aux_aux;
	feasible_traj_points_aux_aux.push_back(feasible_traj_points_aux);
	feasible_traj_points=feasible_traj_points_aux_aux;
	
	vector<float> traj_stations_aux;
	traj_stations_aux.push_back(0);
	traj_stations=traj_stations_aux;
	
	vector<int> obstruction_here_aux;
	obstruction_here_aux.push_back(0);
	obstruction_here=obstruction_here_aux;
	
	max_feasible_station=0;
	
	lane_blocked=0;
	
	first_sampling_index=-1;
	
	vector<float> offset_combinations_aux;
	offset_combinations_aux.push_back(0);
	offset_combinations=offset_combinations_aux;
	
	num_stations=0;
	num_offsets=0;
	
	C_SamplePoint initial_point_aux;
	initial_point=initial_point_aux;
	
	initial_point_GP_path_i=-1;
	
	resolution=0;
	
	C_StaticObs static_obs_aux;
	static_obs=static_obs_aux;
	
	
} 




//funcions membre

void C_TrajGen::generate_initial_opt (float r_col, float min_turn_r, vector<C_Traj> &initial_trajs)
{
	// SI NO HI HA CAP OBSTACLE EN LA ZONA DE DISCRETITZACIO FINA --> DIRECTAMENT CREEM LES TRAJECTORIES FINS A LA ULTIMA 'STATION' D'AQUESTA DISCRETITZACIO FINA
	if (obstruction_here[first_sampling_index]==0) // % If there is NO obstruction in the initial sampling ( no hi ha cap obstruccio en la discretitzacio 'fina')
	{
		float i = first_sampling_index; // index del vector 'traj_stations' que indica on s'acaba la discretització fina
		
		// Generacio de les trajectories amb les corves G2_Spline
		float ini_point_sampling_coords[2]={-1,-1}; // punt inicial de indexos de la matriu (el -1 -1 indica que el cotxe es troba just a fora de la matriu de mostreig)
		float end_point_sampling_coords[2]; // numero de 'station' i 'offset' on es troba el 'end_point' de la trajectora
		vector <vector<float> > traj; // trajectoria amb els vectors de les coordenades 'x', 'y' i 'curvatures'
		float dist;  // longitud de la trajectoria
		
		//Desde el punt inicial on ens trobem calculem les diferents trajectories als diversos offsets de la ULTIMA 'station' de la discretitzacio fina
		for (int j=0; j<num_offsets; j++) //PER CADA OFFSET --> (LA STATION ES SEMPRE LA MATEIXA ( la ultima de la discretitzaio fina feta)
		{
			vector<float> Px; // conjunt coordenades X de de la trajectoria
			vector<float> Py; // conjunt coordenades Y de de la trajectoria
			vector<float> Pk; // conjunt de curvaturesd de la trajectoria
			
			G2_Spline(initial_point.x, initial_point.y, initial_point.k, initial_point.head*(M_PI/180), traj_points[i][j].x, traj_points[i][j].y, traj_points[i][j].k, traj_points[i][j].head*(M_PI/180), 5, resolution, Px, Py, dist, Pk);			
			
			// Construccio del elelemnt de 'C_traj' que reresentara una de les trajectories 
			//columna i fila de la matriu --> ('station i offset') (contrari matlab)
			end_point_sampling_coords[0]=i; // (numero de station) 
			end_point_sampling_coords[1]=j; // (numero de offset)
			
			traj.push_back(Px);
			traj.push_back(Py);
			traj.push_back(Pk);
			
			C_Traj aux (ini_point_sampling_coords, end_point_sampling_coords, traj, dist, resolution);
			
			//Afegim la trajectoria creada al vector que conte les possibles trajectories a executar
			initial_trajs.push_back(aux);
			
			traj.clear();
			
		}
		
	}
	else //% There is obstruction in the initial sampling --> hi ha un obstacle en la discretitzacio fina feta --> haure de crear totes les trajecories viables d'aquella discretitzacio
	{
		vector<float> feas_offsets_indexs; // vector amb els indexs dels vectors de offsets de la primera 'station' que son accesibles (tenen un 1)		
		
		for (int i=0; i<(int)feasible_traj_points[0].size(); i++)
		{
			if (feasible_traj_points[0][i]==1)
			{
				feas_offsets_indexs.push_back(i);
			}
		}
		
		vector <C_Obstacle> closest_obs;
		float obs_station;
		int no_obstacle; 
		
		
		int num_feas_offsets=feas_offsets_indexs.size(); // numero de offsets disponibles a la primera 'station'
		int num_trajs = (first_sampling_index+1)*num_feas_offsets; // numero_trajectories_generades= num_stations*numero_offsetsDisponibles_1a_station // % Because after the first_sampling_index, the lateral offsets are CONSTANT
		
		
		// Obtinc el obstacle/s visible/s mes propers de la primera 'station' que li queden per davant 
		static_obs.get_closest_obs_from_pos(initial_point_GP_path_i, closest_obs, obs_station, no_obstacle); 
		
		//definicio de les varibles de cost de la trajectoria (definicio en pg 61 TFM) 
		//vectors dels costos asociats, cada component del vector pertany a una trajectoria determinada
		vector <float> c_lengths;
		vector <float> c_maxks;
		vector <float> c_maxkdots;
		vector <float> c_offsets;
		vector <float> c_static_obs;
		//inicialitzem tots els costos a 0
		for (int i=0; i<num_trajs; i++)
		{
			c_lengths.push_back(0);
			c_maxks.push_back(0);
			c_maxkdots.push_back(0);
			c_offsets.push_back(0);
			c_static_obs.push_back(0);
		}
		
		vector<C_Traj> total_trajs;
		int count=0;
		
		for (int i=0; i<(first_sampling_index+1); i++) // PER A CADA STATION DE LA DISCRETITZACIO FINA
		{
			for (int jj=0; jj<num_feas_offsets; jj++) // PER A OFFSET ACCESIBLE
			{
				float j=feas_offsets_indexs[jj];
			
				vector<float> Px; // conjunt coordenades X de de la trajectoria
				vector<float> Py; // conjunt coordenades Y de de la trajectoria
				float dist;  // longitud de la trajectoria
				vector<float> Pk; // conjunt de curvaturesd de la trajectoria
			
				G2_Spline(initial_point.x, initial_point.y, initial_point.k, initial_point.head*(M_PI/180), traj_points[i][j].x, traj_points[i][j].y, traj_points[i][j].k, traj_points[i][j].head*(M_PI/180), 5, resolution, Px, Py, dist, Pk);			
			
				if (i<first_sampling_index) // % if it's not the last station of the thin discretitzation, we need to generate the rest of the path
				{
					
					// Generem trajectoria desde el punt [station, offset] enq ue ens trobem fins al offset corresponent de la ultima 'station' de la discretitzacio fina
					vector<float> Px2; // conjunt coordenades X de de la trajectoria
					vector<float> Py2; // conjunt coordenades Y de de la trajectoria
					float dist2;  // longitud de la trajectoria
					vector<float> Pk2; // conjunt de curvaturesd de la trajectoria
					
					G2_Spline(traj_points[i][j].x, traj_points[i][j].y, traj_points[i][j].k, traj_points[i][j].head*(M_PI/180), traj_points[first_sampling_index][j].x, traj_points[first_sampling_index][j].y, traj_points[first_sampling_index][j].k, traj_points[first_sampling_index][j].head*(M_PI/180), 5, resolution, Px2, Py2, dist2, Pk2);			
					
										
					float ini_point_sampling_coords[2]={-1,-1}; // punt inicial de indexos de la matriu / el -1 -1 indica que el cotxe es troba just a fora de la matriu de mostreig 
					float end_point_sampling_coords[2]; // numero de 'station' i 'offset' on es troba el 'end_point' de la trajectora
					end_point_sampling_coords[0]=first_sampling_index; // (station) 
					end_point_sampling_coords[1]=j; // offset
					
										
					// Construim trajectories senceres
					vector<float> Pxtot; // conjunt coordenades X de de la trajectoria
					vector<float> Pytot; // conjunt coordenades Y de de la trajectoria
					float disttot;  // longitud de la trajectoria
					vector<float> Pktot; // conjunt de curvaturesd de la trajectoria
					
					Pxtot=Px;
					Pytot=Py;
					disttot=dist+dist2;
					Pktot=Pk;
										
					for (int s=0; s<(int)Px2.size(); s++) 
					{
						Pxtot.push_back(Px2[s]);
						Pytot.push_back(Py2[s]);
						Pktot.push_back(Pk2[s]);
					}
										
					vector <vector<float> > traj_tot; // trajectoria amb els vectors de les coordenades 'x', 'y' i 'curvatures'

					traj_tot.push_back(Pxtot);
					traj_tot.push_back(Pytot);
					traj_tot.push_back(Pktot);
					
					C_Traj aux_CTraj (ini_point_sampling_coords, end_point_sampling_coords, traj_tot, disttot, resolution);
					
					total_trajs.push_back(aux_CTraj); // afegim trajectoria al conjunt de trajectories
					
					traj_tot.clear();
										
				}
				else // cas per la ultima 'station'
				{
					float ini_point_sampling_coords[2]={-1,-1}; // punt inicial de indexos de la matriu / el -1 -1 indica que el cotxe es troba just a fora de la matriu de mostreig 
					float end_point_sampling_coords[2]; // numero de 'station' i 'offset' on es troba el 'end_point' de la trajectora
					end_point_sampling_coords[0]=first_sampling_index; // (station) 
					end_point_sampling_coords[1]=j; // offset
					
					vector <vector<float> > traj_2; // trajectoria amb els vectors de les coordenades 'x', 'y' i 'curvatures'

					traj_2.push_back(Px);
					traj_2.push_back(Py);
					traj_2.push_back(Pk);
					
					C_Traj aux_CTraj (ini_point_sampling_coords, end_point_sampling_coords, traj_2, dist, resolution);
					
					total_trajs.push_back(aux_CTraj); // afegim trajectoria al conjunt de trajectories
					
					traj_2.clear();
				}
				
				c_lengths[count]=total_trajs[count].len;
				c_maxks[count] = total_trajs[count].maxk;
				c_maxkdots[count] = total_trajs[count].maxkdot;
				c_offsets[count] = j;
				
				float c_static_obs1;
				float col_index1;
				
				total_trajs[count].compute_cost_static_obs(closest_obs, r_col, c_static_obs1, col_index1);
				
				c_static_obs[count]= c_static_obs1;
				
								
				count = count + 1;
			}
						
		}
				
		// Normalitzacio els valors dels vectors de costos 
		
		vector<float> c_offsets_; // aquest no es modificara
		
		for (int i=0; i<(int)c_lengths.size(); i++) 
		{
			c_lengths[i]=c_lengths[i]/((traj_stations[first_sampling_index]+1)*resolution); // cost=(longitud trajectoria/longitud de la trajectoria d'offset 0)
			c_maxks[i]=c_maxks[i]/(1/min_turn_r);
			c_maxkdots[i]=c_maxkdots[i]/(1/min_turn_r);
			c_offsets_.push_back(c_offsets[i]/num_offsets);
		}
		
		vector<float> total_cost; // vector conte els costos totals de cada possible trajectoria
		
		//pesos
		float wl, wk, wkdot, woff, wobs_s;
		wl=0.5;
		wk=1;
		wkdot=1;
		woff=0.1;
		wobs_s=1;
		
		float cost_aux;
		for (int i=0; i<(int)c_lengths.size(); i++) 
		{
			cost_aux=(wl*c_lengths[i])+(wk*c_maxks[i])+(wkdot*c_maxkdots[i])+(woff*c_offsets_[i])+(wobs_s*c_static_obs[i]);
			total_cost.push_back(cost_aux); //afegim el valor al vectos de costos
		}
		
		vector<float> indexs;
		
		// recorro els offsets de la matriu per veure de cada offset quina es la trajectoria associada amb minim cost
		for (int jj=0; jj<num_feas_offsets; jj++) 
		{
			int j=feas_offsets_indexs[jj];
			
			// posem a 1 els valors diferents al offset actual
			for (int ii=0; ii<(int)c_offsets.size(); ii++) 
			{ 
				if (c_offsets[ii]!=j)
				{
					indexs.push_back(1);
				}
				else
				{
					indexs.push_back(0);
				}
			}
			
			vector<float> total_cost_aux;
			for (int s=0; s<(int)total_cost.size(); s++) 
			{
				total_cost_aux.push_back(total_cost[s]+(10*indexs[s]));
			}
			
			//minim cost
			float min_cost; 
			min_cost= min_value(total_cost_aux); 
			//busquem index asociat al minim cost
			float traj_i; //index de la trajectoria amb el minim cost asociat
			for (int i=0; i<(int)total_cost_aux.size(); i++) 
			{
				if (total_cost_aux[i]==min_cost)
				{
					traj_i=i;
				}
			}
			
			initial_trajs.push_back(total_trajs[traj_i]); // retorno les trajectoreis no obstruides, d'offset amb minim cost
			
		} 
		 		
	}

}

void C_TrajGen::generate_rest (vector<C_Traj> &trajs)
{
	// Calcul de les trajectories a generar fins la 1a 'station'
	int num_initial_trajs; // numero de trajectotries a generar a l'inici
	vector<float> feas_offsets_indexs; // offsets disponibles a la primera 'station'
		
	if (obstruction_here[first_sampling_index]==0) //no hi ha obstruccio a la 'station' del final de la discretitzacio fina % If there is no obstruction in the initial sampling
	{
		// tots els offsets estaran disponibles
		for (int i=0; i<num_offsets; i++) 
		{ 
			feas_offsets_indexs.push_back(i);
		}
		num_initial_trajs=num_offsets; // hi hauran tantes trajectories inicials com a offsets
	}
	else //% There is obstruction in the initial sampling
	{
		for (int i=0; i<(int)feasible_traj_points[0].size(); i++)
		{
			if (feasible_traj_points[0][i]==1) // mirem offets de la 1a 'station'
			{
				feas_offsets_indexs.push_back(i); // offsets disponibles a la primera station
			}
		}
		num_initial_trajs=feas_offsets_indexs.size();
	}
	
	
	vector<float> last_js = feas_offsets_indexs; //per a guardar l'anterior vector on s'indiquen els indexs dels offsets disponibles
	
	
	//crem vector de trajectories per defecte
	for (int i=0; i<num_initial_trajs; i++) // incialitzacio de les trajectories --> hi hauran tantes com a offsets disponibles
	{ 
		C_Traj aux;
		
		trajs.push_back(aux);
	}
			
	for (int i=(first_sampling_index+1); i<num_stations; i++) // PER A CADA STATION començant per la 1a del mostreig ampliat (la seguent a la de la discretitzacio fina) --> ( del mostreig fi ja em tret les trajectories amb 'generate_initial_opt()' )
	{ 
		
		vector<float> end_feas_offsets_indexs; //vector per emmagatzemar els offsets accesibles de la 'station' tractada
		float target_j; // numero de offset escollit per passar d'aquella 'station'
		
		for (int j=0; j<(int)feasible_traj_points[i].size(); j++) // PER CADA OFFSET 'j' de la 'station' 'i' tractada
		{
			if (feasible_traj_points[i][j]==1) // mirem offets de la 1a 'station'
			{
				end_feas_offsets_indexs.push_back(j); // offsets disponibles 'j' de la station 'i' tractada
			}
		}
		
		if ((int)end_feas_offsets_indexs.size() < (num_offsets) ) // Algun offset esta obstruit --> % Obstruction here, propagated or not
		{
			// Busquem el punt mes allunyat de l'obstacle --> Quan hi ha obstacle sempre pasarem pel punt mes allunyat a aquest 
			
			if ( (feasible_traj_points[i][0]==0) && (feasible_traj_points[i].back()==1) ) //cas obstacle es troba a la part esquerra del carril
			{
				target_j = num_offsets-1; // el offset per on pasarem sera el de mes a la dreta del carril
			}   
			else if ( (feasible_traj_points[i][0]==1) && (feasible_traj_points[i].back()==0) ) //cas obstacle es troba a la part dreta del carril
			{
				target_j = 0; // // el offset per on pasarem sera el de mes a la esquerra del carril
			}
			else // cas en que hi ha obstruccio a abdos costats de la carretera
			{
				target_j= end_feas_offsets_indexs[ceil(end_feas_offsets_indexs.back()/2)]; // agafem el offset que queda enmig dels que hi ha disponibles
			}
			
		}
			
				
		float last_j; // ultim offset escollit
		
		for (int jj=0; jj<num_initial_trajs; jj++) // per a cada trajectoria a generar
		{
			last_j=last_js[jj]; // ultim offset escollit de la trajectoria anterior
			
			if ( sum(offset_combinations) == 0 ) // Si el vector de combinations es tot 0s, emprem el offset emprat a la 'station' anterior --> % Only maintain offset
			{
				vector<float> Px; // conjunt coordenades X de de la trajectoria
				vector<float> Py; // conjunt coordenades Y de de la trajectoria
				float dist;  // longitud de la trajectoria
				vector<float> Pk; // conjunt de curvaturesd de la trajectoria
				
				G2_Spline(traj_points[i-1][last_j].x, traj_points[i-1][last_j].y, traj_points[i-1][last_j].k, traj_points[i-1][last_j].head*(M_PI/180), traj_points[i][last_j].x, traj_points[i][last_j].y, traj_points[i][last_j].k, traj_points[i][last_j].head*(M_PI/180), 5, resolution, Px, Py, dist, Pk);			
					
				if (i==first_sampling_index + 1) // primera station tractada de la discretitzacio ampliada
				{
					
					float ini_point_sampling_coords[2]={float(i-1),last_j}; // punt inicial de indexos de la matriu
					float end_point_sampling_coords[2]; // numero de 'station' i 'offset' on es troba el 'end_point' de la trajectora
					vector <vector<float> > traj; // trajectoria amb els vectors de les coordenades 'x', 'y' i 'curvatures'
					
										
					end_point_sampling_coords[0]=i; // (station) 
					end_point_sampling_coords[1]=last_j; // offset
			
					traj.push_back(Px);
					traj.push_back(Py);
					traj.push_back(Pk);
			
					C_Traj aux (ini_point_sampling_coords, end_point_sampling_coords, traj, dist, resolution);
			
					//Afegim la trajectoria creada al vector que conte les possibles trajectories a executar
					trajs[jj]=aux;
					traj.clear();
				}
				else //resta de 'stations' , afegim els punts als de la trajectoria anterior
				{
					vector <vector<float> > new_traj;
					new_traj= trajs[jj].traj;
										
					for (int ii=0; ii<(int)Px.size(); ii++)
					{
						new_traj[0].push_back(Px[ii]);//afegim coord x's
						new_traj[1].push_back(Py[ii]);//afegim coord y's
						new_traj[2].push_back(Pk[ii]);//afegim coord k's
					}		
					
					float new_dist;
					new_dist=trajs[jj].len+dist;
					
					float ini_point_sampling_coords[2]={first_sampling_index, last_j}; // punt inicial de indexos de la matriu
					float end_point_sampling_coords[2]; // numero de 'station' i 'offset' on es troba el 'end_point' de la trajectora
								
					end_point_sampling_coords[0]=i; // (station) 
					end_point_sampling_coords[1]=last_j; // offset
						
					C_Traj aux (ini_point_sampling_coords, end_point_sampling_coords, new_traj, new_dist, resolution);
					// Afegim trajectoria creada			 
					trajs[jj]=aux;
					new_traj.clear();
				}
			}
			else // cas vector de combinations no esta buit --> cal anirem al offset mes allunyat del obstacle / %Go to the farthest offset from the obstacle (this eliminates unnecessary combinations)
			{
				vector<float> Px; // conjunt coordenades X de de la trajectoria
				vector<float> Py; // conjunt coordenades Y de de la trajectoria
				float dist;  // longitud de la trajectoria
				vector<float> Pk; // conjunt de curvaturesd de la trajectoria
				
				// construiem trajectoria fins el offset 'tarjet_j' deduit anteriormetn (cas amb obstacles)
				G2_Spline(traj_points[i-1][last_j].x, traj_points[i-1][last_j].y, traj_points[i-1][last_j].k, traj_points[i-1][last_j].head*(M_PI/180), traj_points[i][target_j].x, traj_points[i][target_j].y, traj_points[i][target_j].k, traj_points[i][target_j].head*(M_PI/180), 5, resolution, Px, Py, dist, Pk);			
				
				if (i==first_sampling_index+1) // cas pel primer cop
				{
					float ini_point_sampling_coords[2]={float(i-1),last_j}; // punt inicial de indexos de la matriu
					float end_point_sampling_coords[2]; // numero de 'station' i 'offset' on es troba el 'end_point' de la trajectora
					vector <vector<float> > traj; // trajectoria amb els vectors de les coordenades 'x', 'y' i 'curvatures'
															
					end_point_sampling_coords[0]=i; // (station) 
					end_point_sampling_coords[1]=target_j; // offset
			
					traj.push_back(Px);
					traj.push_back(Py);
					traj.push_back(Pk);
			
					C_Traj aux (ini_point_sampling_coords, end_point_sampling_coords, traj, dist, resolution);
			
					//Afegim la trajectoria creada al vector que conte les possibles trajectories a executar
					trajs[jj]=aux;
					traj.clear();
				}
				else // quan no es el primer cop
				{
					float ini_point[2]; 
					ini_point[0]=trajs[jj].ini_point_sampling_coords[0];
					ini_point[1]=trajs[jj].ini_point_sampling_coords[1];
					
					float end_point[2];
					end_point[0]=i;
					end_point[1]=target_j;
					
					vector <vector<float> > new_traj;
					new_traj= trajs[jj].traj;
										
					for (int ii=0; ii<(int)Px.size(); ii++)
					{
						new_traj[0].push_back(Px[ii]);//afegim coord x's
						new_traj[1].push_back(Py[ii]);//afegim coord y's
						new_traj[2].push_back(Pk[ii]);//afegim coord k's
					}		
					
					float new_dist;
					new_dist=trajs[jj].len+dist;
					
					
					C_Traj aux (ini_point, end_point, new_traj, new_dist, resolution);
					// Afegim trajectoria creada			 
					trajs[jj]=aux;
					new_traj.clear();
					
				}
				
				last_js[jj]=target_j;
				
							
			}
		}
	}
}	


void C_TrajGen::merge (vector<C_Traj> initial_trajs, vector<C_Traj> rest_trajs, float r_col, vector<C_Traj> &trajs, vector<float> &c_lengths, vector<float> &c_maxks, vector<float> &c_maxkdots, vector<float> &c_offsets, vector<float> &c_static_obs, vector<float> &collisions)
{
	int num_trajs=initial_trajs.size();
	
	for (int i=0; i<num_trajs; i++) //inicialitzo vector de trajectories combinades
	{
		C_Traj aux;
		trajs.push_back(aux);
	}
	
	// % List of closest obstacles
	vector <C_Obstacle> closest_obs;
	vector <float> obs_station;
	int no_obstacle;
	
	// Obstacles del carril -->  % initial station excluded, final station included
	static_obs.get_all_closest_obs_interval( initial_point_GP_path_i, (initial_point_GP_path_i + max_feasible_station), closest_obs, obs_station, no_obstacle); // retorna un vector de tots els obstacles visibles i ordenats de mes aprop a més lluny i dins un interval de 'stations' indicat pel valors de 'station' major i el valor de 'station' menor tambe retorna el vector associat a les estacions d'aquests objectes (tambe ordenat) i una variable per indicar quan no hi ha cap objecte visible 

	//inicialitzem tots els costos a 0
	for (int i=0; i<num_trajs; i++)
	{
		c_lengths.push_back(0);
		c_maxks.push_back(0);
		c_maxkdots.push_back(0);
		c_static_obs.push_back(0);
		c_offsets.push_back(0);
		collisions.push_back(-1); // % For knowing if collision. -1 if not, otherwise it has the target_i of the collision (el -1 indica que no hi ha colisio a aquella station)
	}
	
	if (max_feasible_station > traj_stations[first_sampling_index]) // % If the first sampling is not blocked
	{
		for (int i=0; i<num_trajs; i++)
		{
			// punts inicials i finals de la trajectoria complerta
			float ini_point[2];
			ini_point[0]=initial_trajs[i].ini_point_sampling_coords[0];
			ini_point[1]=initial_trajs[i].ini_point_sampling_coords[1];
			
			float end_point[2];
			end_point[0]=rest_trajs[i].end_point_sampling_coords[0];
			end_point[1]=rest_trajs[i].end_point_sampling_coords[1];
			
			
			//creacio de la nova trajectoria total (unim coordenades de la inicial i la resta)
			vector <vector<float> > new_traj;
			
			new_traj=initial_trajs[i].traj;
				
			for (int j=0; j<(int)rest_trajs[i].traj[0].size(); j++)
			{
				new_traj[0].push_back(rest_trajs[i].traj[0][j]);
				new_traj[1].push_back(rest_trajs[i].traj[1][j]);
				new_traj[2].push_back(rest_trajs[i].traj[2][j]);
			}
			
			//longitud total
			float new_dist = initial_trajs[i].len+rest_trajs[i].len;
			
			// creacio de la trajectoria total
			C_Traj aux (ini_point, end_point, new_traj, new_dist, resolution);
			
			// Afegim trajectoria creada			 
			trajs[i]=aux;
			new_traj.clear();
			
			// Asociem costos de la trajectoria
			c_lengths[i] = trajs[i].len;
            c_maxks[i] = trajs[i].maxk;
            c_maxkdots[i] = trajs[i].maxkdot;
			
						
			float c_static_obs_aux;
			float col_index;
			
			trajs[i].compute_cost_static_obs(closest_obs, r_col, c_static_obs_aux, col_index); //calcula es cost estatic total asociat de la trajectoria respecte els diferent obstacles que hi han
			c_static_obs[i] = c_static_obs_aux;
			
			if (col_index>(-1)) // quan val -1 es NO hi ha colisio, de lo contrari, n'hi hi ha (esta indicant el index de la 'station' on hi ha
			{
				collisions[i] = col_index; //indiquem que hi ha colisio posant el seu index
			}
			
			for (int j=0; j<num_offsets; j++) // % Search for the offset of the ending point of the traj
			{
					if ( feasible_traj_points[feasible_traj_points.size()-1][j]==1 ) // mirem els offsets de la ultima 'station'
					{
						if ( ((num_stations-1)==trajs[i].end_point_sampling_coords[0]) && (j==trajs[i].end_point_sampling_coords[1]) ) //cas en que arrivem al punt últim del mostreig espaial (ultima station i ultim offset)
						{
							c_offsets[i] = j;
							break;
						}
						
					}
			}
		}
	}
}

void C_TrajGen::best_path (vector<C_Traj> trajs, vector<float> c_lengths, vector<float> c_maxks, vector<float> c_maxkdots, vector<float> c_offsets, vector<float> c_static_obs, vector<float> collisions, float min_turn_r, C_Traj &best_traj, float &min_cost, float &best_traj_col, float &traj_i)
{
	// Normalitzacio els valors dels vectors de costos 
		
	vector<float> c_lengths_norm;
	vector<float> c_maxks_norm;
	vector<float> c_maxkdots_norm;
	vector<float> c_offsets_norm;
	 
	
	//inicialitzacio costos totals a 0
	for (int i=0; i<(int)trajs.size(); i++)
	{
		c_lengths_norm.push_back(0);
		c_maxks_norm.push_back(0);
		c_maxkdots_norm.push_back(0);
		c_offsets_norm.push_back(0); // en normalitzar, el offset pertanyent a la trajecorria central del carril tindra cost 0, i la resta cada cop mes, a mesura que s'allunyin de la trajectoria central
	}
	
	vector<float> total_cost; 
	float total_cost_aux;
	
	for (int i=0; i<(int)c_lengths.size(); i++) 
	{
		c_lengths_norm[i]=c_lengths[i]/((traj_stations.back()+1)*resolution); // cost=(longitud trajectoria/longitud de la trajectoria d'offset 0)
		c_maxks_norm[i]=c_maxks[i]/(1/min_turn_r);
		c_maxkdots_norm[i]=c_maxkdots[i]/(1/min_turn_r);
		c_offsets_norm[i]=abs( c_offsets[i]-(ceil(num_offsets/2)) ); // % Offset from the center, in index
		
		total_cost_aux= c_lengths_norm[i]+c_maxks_norm[i]+c_maxkdots_norm[i]+c_offsets_norm[i] + c_static_obs[i];
		total_cost.push_back(total_cost_aux);
	}
	
			
	min_cost=min_value(total_cost);
	
	for (int i=0; i<(int)total_cost.size(); i++)
	{
		if (total_cost[i]==min_cost)
		{
			traj_i=i; // agafem la primera trajectoria que tingui aquell cost minim trobat
			break;
		}
	}
	
	best_traj=trajs[traj_i]; // trjectoria optima escollida (de cost asociat menor)
	best_traj_col=collisions[traj_i]; // per indicar si amb aquella trajectoria es produeix colisio en alguna 'station'
}

void C_TrajGen::compute_vel_static_restrictions (C_Traj traj, float our_station, float collision, float min_turn_r, float comfort_a_lat, vector <vector<float> > &vel_restr)
{
	
	vector<float> vel_restr_aux;
	
	//inicialitzem a 0 els valors de vel_restr
	for (int i=0; i<2; i++)
	{
		vel_restr_aux.push_back(0);
		vel_restr_aux.push_back(0);
		vel_restr.push_back(vel_restr_aux);
		vel_restr_aux.clear();
	}
	
	//vel_restr_aux.clear();
	
	vector<float > traj_k_aux( &traj.traj[2][our_station], &traj.traj[2][traj.traj[2].size()]); // Afafem nomes el conjunt de curvatures que ens queden per davant % This way, we filter if the absolute maximum point is behind us
	
	abs( traj_k_aux); // transformo el vector de curvatures en valors positius
	float c_maxk = max_value( traj_k_aux); // busco el maxim valor de curvatura en valor absolut
	
	
	float max_k_index; // index del punt de trajectoria corresponent a la major curvatura
	for (int i=0; i<(int)traj_k_aux.size(); i++)
	{
		if (traj_k_aux[i]==c_maxk)
		{
			max_k_index=i;
			break;			
		}
	}
	
	
	float max_k_dist = (max_k_index+1)*resolution; //distancia longitudinal a la que es produeix la maxim curvatura (metres)
		
	float collision_k; //indica amb un 1 si es produeix colisio per curvatura, sino val 0
	float max_vel_admissible; // maxima velocitat admisible degut a la curvatura mes pronunciada existent
	
		
	if ( c_maxk >= (1/min_turn_r) )// cas en que la curvatura que es presenta es superior a la capacitat de gir del cotxe --> es produeix collision per curvatura massa prununciada
	{
		collision_k = 1; //% Collision by curvature
	}
	else
	{
		collision_k = 0;
		
		if (c_maxk == 0)
		{
			max_vel_admissible=10000;
		}
		else
		{
			max_vel_admissible = sqrt(comfort_a_lat/c_maxk);
		}
        
        vel_restr[0][0]=max_k_dist;
        vel_restr[0][1]=max_vel_admissible;
    }
   	
   	   	
   	if (collision>-1) // cas en que ja es produia colisio per obstacle
   	{
		if (collision_k==1) // tambe es produeix colisio per curvatura
		{
			vel_restr[1][0]=min(max_k_dist, resolution*(collision+1)) ;
			vel_restr[1][1]=0;
		}
		else
		{
			vel_restr[1][0]=resolution*(collision+1);
			vel_restr[1][1]=0;
		}

	}
}

void C_TrajGen::compute_best_vp ( C_Traj traj, float our_station, vector <vector<float> > vel_restr, float max_vel_restriction[2], float vel, float max_road_vel, float a_0, float a_ant, float max_decel, float max_accel, float current_t, float delta_t, vector<C_DynObstacle> dyn_obs, float r_col, float lookahead_dist, vector<vector<float> > &best_vp, float &best_vp_cost, float &vp_accel, float &vp_finalvel)
{
	vector <vector<vector<float> > > vel_profiles;
	// costros dinamics
	vector<float> c_maxaccels; // cost dinamic per acceleracio
	vector<float> c_finalvels; // cost dinamic per velocitat
	
	compute_vel_profiles (vel_restr, max_vel_restriction, vel, max_road_vel, a_0, a_ant, max_decel, max_accel, current_t, delta_t, lookahead_dist, vel_profiles, c_maxaccels, c_finalvels);

	int num_vel_profiles = vel_profiles.size(); // numero de perfils de velocitat candidats que tenim
	
	vector<float> c_dynamic_obs; // cost dinamic associat a la repulsio dels obstacles dinamics
	
	for (int i=0; i<num_vel_profiles; i++) // inicialitzacio costos per repulsio obstacles dinamics
	{
		c_dynamic_obs.push_back(0);
	}
	
	int num_dyn_obs = dyn_obs.size();
	
	for (int d=0; d<num_dyn_obs; d++) // PER A CADA OBSTACLE DINAMIC
	{
		if ( dyn_obs[d].visible == 1 ) // es troba visible el obstacle dinamic
		{
			for (int p=0; p<num_vel_profiles; p++) // PER A CADA PERFIL DE VELOCITAT CANDIDAT
			{
				int num_points = vel_profiles[p][0].size(); //numero de punts del perfil de velocitat que tenim
				float min_dist = 9999999;
				float target_i = 9999999;
				
				int sampling_size=10; // (PARAMETRE) indica cada quants punts mirem el perfl de velocitats
				
				for (int i=0; i<num_points; i=i+sampling_size) // MIREM 1 DE CADA 10 PUNTS DEL PERFIL DE VELOCITAT 
				{
					float t= vel_profiles[p][3][i]; // mirem el intant de temps
					
					// mirem si el obstacle es mou en aquell moment % t inside the movement of the dynamic obstacle
					if (t <= dyn_obs[d].positions[2].back()) // ( encara no s'ha aturat el obstacle dinamic )
					{
						if (t >= dyn_obs[d].positions[2][0]) // // ja s'ha posat en marxa el obstacle dinamic
						{
							float station = round( (vel_profiles[p][0][i]/resolution) ) -1 + our_station; // 'index de punt associat al obstacle dinamic' en aquell instant
							
							if ( station > (traj.traj[0].size()-1) )
							{
								break;
								station=traj.traj[0].size()-1;
							}
							else if (station < 0)
							{
								station=0;
							}
							
							float t_= round(t/delta_t)*delta_t; // % Multiple of delta_t
							float index=-1; 
							
							for (int j=0; j< (int) dyn_obs[d].positions[2].size(); j=j+sampling_size) // busquem index de carril associat al instant de temps
							{
								if ( dyn_obs[d].positions[2][j]==t_ )
								{
									index=j;
									break;
								}
							}							
							
							float index_aux = 30; // indexs de mes per aturar la iteracio si index ja trobat
							
							if (index>-1) // hem trobat un index assciat al temps 
							{
								//calculem la distancia de la trajectoria escollida fins al obstacle dinamic
								float dist =sqrt( (pow((traj.traj[0][station] - dyn_obs[d].positions[0][index]),2) ) + (pow((traj.traj[1][station] - dyn_obs[d].positions[1][index]),2) ) );
								
								if (dist < min_dist)
								{
									min_dist=dist;
									target_i=i; // punt del perfil de velocitat
								}
								else if ( i > (target_i+index_aux) )
								{
									break; // % If the minimum is found, there's no need to continue iterating 
								}
							}
							
						}
					}
					else
					{
						break;
					}
				}
				c_dynamic_obs[p] = c_dynamic_obs[p] + dynamic_obs_function( min_dist - dyn_obs[d].r - r_col ); // calculem el cost associat al obstacle
			}
		}
	}
	
	//% COSTS normalization: -------------------------------- (pg 61 TFM)
	vector<float> c_maxaccels_norm;
	
	for (int i=0; i< (int) c_maxaccels.size(); i++) 
	{
		c_maxaccels_norm.push_back( abs(c_maxaccels[i])/abs(max_decel) ); 
	}
	
	vector<float> c_finalvels_norm;
	
	for (int i=0; i< (int) c_finalvels.size(); i++) 
	{
		c_finalvels_norm.push_back( 1 - (c_finalvels[i]/ceil(max_road_vel)) ); 
	}
	
	// Calcul del cost total
	
	vector<float> total_cost; // cost total dinamic asociat a cada perfil de velocitat candidat
	
	//pesos de la funcio de cost
	float w_accel =1; // pes associat a la component de acceleracio (PARAMETRE)
	float w_vel=10; // pes associat a la component de velocitat (PARAMETRE)
	float w_dyn_obs=1; // pes associat als obstacles dinamics (PARAMETRE) 
	
	float cost_aux=0;
	for (int i=0; i< (int) c_finalvels.size(); i++) 
	{
		cost_aux=(w_accel*c_maxaccels_norm[i]) + (w_vel*c_finalvels_norm[i]) + (w_dyn_obs*c_dynamic_obs[i]);
		total_cost.push_back(cost_aux);
	}
	
	//Eleccio perfil de velocitats (minim cost dinamic associat)
	best_vp_cost = min_value( total_cost);
	
	//Index associat 
	float vp_index;
	
	for (int i=0; i< (int) total_cost.size(); i++) 
	{
		if (total_cost[i]==best_vp_cost)
		{
			vp_index=i;
		}
	}
	
	//resta de outputs de la funcio
	best_vp = vel_profiles[vp_index];
    vp_accel = c_maxaccels[vp_index];
    vp_finalvel = c_finalvels[vp_index];
}


void C_TrajGen::compute_vel_profiles (vector <vector<float> > vel_restr, float max_vel_restriction[2], float vel, float max_vel, float a_0, float a_ant, float max_decel, float max_accel_in, float current_t, float delta_t, float lookahead_dist, vector <vector<vector<float> > > &vel_profiles, vector<float> &c_maxaccels, vector<float> &c_finalvels)
{
	float max_accel=max_accel_in;
	
	
	
	//% Check lane geometry restrictions against path restrictions:
	if (max_vel_restriction[0]>0) // distancia longitudinal a partir de la qual no es pot superar la distancia maxima
	{
		if ( round(max_vel_restriction[1]) == round( vel_restr[0][1] ) ) // % Same vels, we pick the closest distance
		{
			vel_restr[0][0]=min(vel_restr[0][0], max_vel_restriction[0]); // agafem la distancia mes petita de les dues restriccions
		}
		else if ( max_vel_restriction[1] < vel_restr[0][1] ) // ja era mes restrictiva la 1a
		{
			vel_restr[0][1]=max_vel_restriction[1]; // agafem la distancia del cas de velocitat mes restrictiva
		}
	}
	
	 float vels_increment = 0.5; //(PARAMETRE) , magnitud en que es diferenciaran les velocitats finals de cada perfil de velocitat generat
	 vector<float> target_vels; // vector amb les velocitats finals de cada un dels perfils de velocitat que generarem
	 float security_cofficient=0.9; //(PARAMETRE) coeficient de seguretat per a reduir les distancies de seguretat d'aturarda
	 float distance_collision=2; //(PARAMETRE) metres de seguretat fins a un obstacle [metres]
	 
	 	 
	 //% First of all, check collisions:
	 
	 //Cas es produeix colisio --> crearem perfil de velocitats per a aturar el cotxe abans on es produeix la colisio
	 if (vel_restr[1][0]>0) //% The path station in [m] in the 2nd row -->  indica la distancia longitudinal a la que es produira colisio
	 {
		target_vels.push_back(0); //la velocitat final objectiu sera 0 (voldrem aturar el cotxe)
		 
		vector<float> Punts; // no usarem
		vector<float> Vels; // no usarem
		vector<float> Accels; //acceleracions a assolir
		vector<float> temps; // no usarem
		 		
		//% We add a security coefficient of 0.9 in the distance to stop
		spline_3rdo_velX(vel, 0, 0, (vel_restr[1][0]*security_cofficient), current_t, delta_t, Punts, Vels, Accels, temps);
		
		//la maxima acceleracio sera la minim de les acceleracions a assolir
		max_accel=min_value(Accels);
	 }
	 else // NO collisions ( no es produeix cap colisio ), no aturarem el cotxe
	 {
		 // % Checking the curvature restrictions
		 if (vel_restr[0][0] > 0) // % The path station in [m] in the 1st row
		 {
			if (vel_restr[0][1]>max_vel) // maxima velocitat per curvatura superior a max_vel --> construim els perfils de velocitat fins a 'max_vel'
			{
				for (float v=0; v<=round(max_vel); v=v+vels_increment)
				{
					target_vels.push_back(v); //creem el vectror amb les velocitats finals de les trajectories
				}
			}
			else
			{
				for (float v=0; v<=round(vel_restr[0][1]); v=v+vels_increment) // el creem fins la maxima velocitat permesa deguda a curvatura
				{
					target_vels.push_back(v); //creem el vectror amb les velocitats finals de les trajectories
				}
				// Find the accelerations
				if ( (vel_restr[0][1]<vel) && (vel_restr[0][2] > distance_collision) ) // % estem a velocitat menor a la maxima i almenys 2 metres fins a obstacle (prevenir errors tecnics)
				{
					vector<float> Punts; // no usarem
					vector<float> Vels; // no usarem
					vector<float> Accels; //acceleracions a assolir
					vector<float> temps; // no usarem
		 		
					spline_3rdo_velX(vel, vel_restr[0][1], 0, vel_restr[0][0], current_t, delta_t, Punts, Vels, Accels, temps);
					max_accel= min_value(Accels);
				}
			}
			 
		 }
		 else // No collisions and no k restrictions 
		 {
			for (float v=0; v<=round(max_vel); v=v+vels_increment) // el creem fins la maxima velocitat permesa deguda a curvatura
			{
				target_vels.push_back(v); //creem el vectror amb les velocitats finals de les trajectories
			}
		 }
	}
	
		
	//Generacio de les acceleracions emprades pels perfils
	vector<float> target_accels;
	float accels_increment = 0.25; 
	
	if (max_accel==0)
	{
		max_accel = -0.1; // ?¿?¿?¿?¿
	}
	
	//creem vector de acceleracions, incorporar un conjunt de acceleracions negatives, pero nomes una positiva, la qual sera la maxima, (motiu ?¿¿?)
	if (max_accel > 0) //acceleracio maxima es positiva
	{
		for (float a=max_decel; a<-0.1; a=a+accels_increment) // max_decel sera un valor negatiu
		{
			target_accels.push_back(a); //creem vector de acceleracions, incorporar un conjunt de acceleracions negatives, pero nomes una positiva, la qual sera la maxima, (motiu ?¿¿?)
		}
	}
	else //acceleracio maxima es negativa
	{
		if (max_accel >= max_decel)
		{
			for (float a=max_decel; a<=max_accel; a=a+accels_increment) // max_decel sera un valor negatiu
			{
				target_accels.push_back(a); //creem vector de acceleracions (tot seran acceleracions negatives)
			}
		}
		else
		{
			target_accels.push_back(max_decel);
		}
	}
	
		
	float num_target_vels = target_vels.size();
	float num_target_accels = target_accels.size();
    float num_profiles= num_target_vels* num_target_accels; // numero total de perfils generats, per cada velocitat final tindrem els perfils de les diferents acceleracions per arrivar a tal velocitat final
	
	// inicialitzacio dels outputs --> tot a 0
	vector <vector<vector<float> > > vel_profiles_;
	vector <vector<float> > vel_restr_aux; 
	vector<float> vel_restr_aux_aux;
	
	for (int i=0; i<(int)num_profiles; i++)
	{
		for (int j=0; j<4; j++) // creem 4 vectors (X's, V's, A's, t's)
		{
			for (int k=0; k<100; k++) // generarem 100 (?¿?) valors per a cada componen (X's, V's, A's, t's)
			{
				vel_restr_aux_aux.push_back(0);
			}
			vel_restr_aux.push_back(vel_restr_aux_aux);
			vel_restr_aux_aux.clear();
		}
		vel_profiles_.push_back(vel_restr_aux);
		vel_restr_aux.clear();
	}
	
	vector<float> c_maxaccels_; 
	vector<float> c_finalvels_; 
	
	for (int i=0; i<(int)num_profiles; i++)
	{
		c_maxaccels_.push_back(0);
		c_finalvels_.push_back(0);
	}
	
	// Generacio de tots els perfils
	
	int count=0;
	
	for (int v=0; v<(int)num_target_vels; v++) // PER A CADA VELOCITAT FINAL
	{
		float vel_diff= target_vels[v] - vel;
		
		for (int a=0; a<(int)num_target_accels; a++) // PER A CADA ACCELERACIO EMPRADA PER ARRIBAR A LA VELOCITAT FINAL
		{
			vector<float> Punts; 
			vector<float> Vels; 
			vector<float> Accels; 
			vector<float> temps; 
			
			float var_acc=2; //?¿?¿?¿?¿?
			
			spline_3rdo_vel_a0 ( abs(target_accels[a]), abs(target_accels[a]), var_acc, a_0, vel, target_vels[v], 0, current_t, delta_t, lookahead_dist, Punts, Vels, Accels, temps);
			
			if ( ( (vel_diff > 0) && (target_accels[a] > 0) ) || ( ( (vel_diff < 0) && (target_accels[a] < 0) ) ) ) // estamos ya acelerandi cuando queremos llegar a una velocidad superior o estamos ya frenando cuando queremos llegar a una inferior (coherente)  / % Accelerate or brake, coherent
			{
				vel_profiles_[count][0] = Punts;
				vel_profiles_[count][1] = Vels; 
				vel_profiles_[count][2] = Accels;
				vel_profiles_[count][3] = temps;
				
				c_maxaccels_[count] = target_accels[a];
				c_finalvels_[count] = target_vels[v];
				count=count+1;
				
			}
			else if ( vel_diff==0 ) // % Maintain constant velocity
			{
				vel_profiles_[count][0] = Punts;
				vel_profiles_[count][1] = Vels; 
				vel_profiles_[count][2] = Accels;
				vel_profiles_[count][3] = temps;
				
				c_maxaccels_[count] = 0;
				c_finalvels_[count] = target_vels[v];
				count=count+1;
				break; // % Only must be done 1 time
			}
		}
	}
	
	
	//% We need to cut the non-assigned variables
	
	if (num_profiles > count)
	{
		for (int i=0; i<(int)count; i++) 
		{
			vel_profiles.push_back(vel_profiles_[i]);
			c_maxaccels.push_back(c_maxaccels_[i]);
            c_finalvels.push_back(c_finalvels_[i]);
		}
	}
	else
	{
		vel_profiles = vel_profiles_;
		c_maxaccels = c_maxaccels_;
        c_finalvels = c_finalvels_;
	}
	
	
}






