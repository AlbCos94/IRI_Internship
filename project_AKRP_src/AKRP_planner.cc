// AKRP_planner

// Includes (preprocessor directives)
#include <sstream>
#include <iostream>
#include <string> 
#include <stack>
#include <vector>
#include <math.h>
#include <cmath>
#include <fstream> 
#include <algorithm> 

// Includes (functions)
#include "car_circles.h" 
#include "G2_Spline.h" 
#include "spline_vel.h"
#include "write_lane.h" 
#include "write_path.h" 
#include "write_vp.h" 
#include "read_lane.h"
#include "Search_closest_point.h" 
#include "static_obs_function.h" 
#include "dynamic_obs_function.h" 
#include "funcions_mat.h" 
#include "max_admissible_vels.h" 
#include "sampling_dist.h" 
#include "check_col_sample_point.h" 

// Includes (Structures)
#include "S_lane_sampling.h" 
#include "S_lane_sampling_comb.h" 

// Includes (classes)
#include "C_Traj.h" 
#include "C_Track.h" 
#include "C_Obstacle.h"
#include "C_StaticObs.h"  
#include "C_SamplePoint.h" 
#include "C_mandatory_point.h" 
#include "C_DynObstacle.h" 
#include "C_Sampling.h" 
#include "C_TrajGen.h"
#include "C_LaneChange.h"  

using namespace std; 

// CONSTANT VALUES

//Host car ---> dimensions del cotxe per a representarlo com a cercles per mes endavant detectar colisions (pagina 16 (Fig 8) especifica dimensions)
#define car_l 4.45 // longitud del cotxe (metres)
#define car_w 1.85 // amplitud del cotxe (metres)
#define car_r_ovh 0.91 // car rear overhaul / voladis traser cotxe (metres) --> distancia de l'eix traser fins el final del cotxe
#define car_wheelbase 2.61 // longitud del cotxe (metres)
#define min_turn_r 6 // minim radi de gir (metres)
#define car_n1 1 // numero de cercles a partir del qual es representa el cotxe
#define car_n3 3 // numero de cercles a partir del qual es representa el cotxe

//Restriccions
//#define max_road_vel 14 // maxima velocitat a la que es pot anar (metres/segon)

//Environment
//#define resolution 0.1 // metres entre punts  (ull ja estan definits per fer altres proves)
//#define lane_width 3.5 //amplada del carril  

//*************************************************************************//
// main

int main() 
{
	//---------------------------//
	//Representacio del cotxe host (amb vectors)
	
	//inputs 
	float pos_X=0.0; // posicio en X del cotxe
	float pos_Y=0.0; // posicio en Y del cotxe
	float heading=0.0; // orientacio del cotxe
	
	// outputs (seran modificats)
	float car_1circle_r; // radi que tindra el cercle que engloba el cotxe
	vector <vector<float> > A (2, vector<float>(car_n1)); // vector que conte les coordenades en X i en Y del cercle que engloba el cotxe
	
	float car_3circle_r; // radi que tindran el cercles que constitueixen el cotxe
	vector <vector<float> > B (2, vector<float>(car_n3)); // vector que conte les coordenades en X i en Y d'aquests cercles

	// Representacio cercle gran engloba tot el cotxe
	car_circles(car_l, car_w, car_r_ovh, pos_X, pos_Y, heading, car_n1, car_1circle_r, A);
	// Representacio cercles petits que constitueixen el cotxe 
	car_circles(car_l, car_w, car_r_ovh, pos_X, pos_Y, heading, car_n3, car_3circle_r, B); 

		
	//---------------------------//
	//Creacio de la carretera i escriure dades en arxiu (no caldra en el programa final (?¿?), ja que llegira les dades d'un arxiu donat)
	
	float resolution=0.1; // metres entre els punts que discretitzen la carretera
	float lane_width=3.5; // amplada dels carrils [metres]
	
	// creacio carrils 1, 2, 3
	
	//inputs
	//punt inici per cada carril 'Ai'
	float x_A1=0, x_A2=0, x_A3=0;
	float y_A1=0, y_A2=lane_width, y_A3=lane_width*2;
	float K_A1=0, K_A2=0, K_A3=0;
	float theta_A1=0, theta_A2=0, theta_A3=0;
		
	//punt final per cada carril 'Bi'
	float x_B1=600, x_B2=600, x_B3=600;
	float y_B1=0, y_B2=lane_width, y_B3=lane_width*2;
	float K_B1=0, K_B2=0, K_B3=0;
	float theta_B1=0, theta_B2=0, theta_B3=0;
	
	int N=5; //numero interacions pels parametres de la Spline
	
	const char* carril11= "carril1.txt";
	const char* carril22= "carril2.txt";
	const char* carril33= "carril3.txt";
		
	write_lane(carril11, x_A1, y_A1, K_A1, theta_A1, x_B1, y_B1, K_B1, theta_B1, N, resolution);
	write_lane(carril22, x_A2, y_A2, K_A2, theta_A2, x_B2, y_B2, K_B2, theta_B2, N, resolution);
	write_lane(carril33, x_A3, y_A3, K_A3, theta_A3, x_B3, y_B3, K_B3, theta_B3, N, resolution);
	
	//---------------------------//
	//Lectura dades de la carretera
		
	vector <vector<float> > carril1; // Cami que vull seguir --> Representat per matriu de posicions i curvartures --> vector format per tres vectors, un contindra coodenades X, un altre Y i un altre curvatures
	vector <vector<float> > carril2; // Un altre carril
	vector <vector<float> > carril3; // Un altre carril
	
	const char* nom_fitxer1= "carril1.txt"; // Fitxer on estan les dades del carril que vull seguir --> coordenades X, Y i curvatures
	const char* nom_fitxer2= "carril2.txt"; // un altre carril que forma la carretera
	const char* nom_fitxer3= "carril3.txt"; // un altre carril que forma la carretera
	
	//Extrec les dades dels fitxers on estan els carrils per posarles en matrius 'carril_i' (amb x's, y's i k's)
	read_lane(nom_fitxer1, carril1); 
	read_lane(nom_fitxer2, carril2); 
	read_lane(nom_fitxer3, carril3); 
	
	//Estrec les distancies dels carrils 'paths distances'
	float path_dist1;
	float path_dist2;
	float path_dist3;
	
	read_path_dist(nom_fitxer1, path_dist1);
	read_path_dist(nom_fitxer2, path_dist2);
	read_path_dist(nom_fitxer3, path_dist3);
			
	//---------------------------//
	//Aplicacio de la clase de C_Traj per representar els carrils de la carretera --> traject_i
	
	float po[2]={-1,-1};
	float pf[2]={-1,-1};
	
		
	C_Traj traject1 (po,pf,carril1,path_dist1,resolution); //objetos de tipo C_Traj que representen els carrils
	C_Traj traject2 (po,pf,carril2,path_dist2,resolution);
	C_Traj traject3 (po,pf,carril3,path_dist3,resolution);
	
	vector <C_Traj > Lanes; // vector que representa el conjunt de la carretera (format per elements de la classe C_Traj)
	
	Lanes.push_back(traject1);
	Lanes.push_back(traject2);
	Lanes.push_back(traject3);
	
	
	//---------------------------//
	//Aplicacio de la clase de C_Track per representar la carretera 
		
	C_Track Highway (lane_width, Lanes); // variable que representa tota la carretera
	
	
	//Definicio del carril que es vol seguir --> GP_path --> 'current Global Planner path' (aquest es el carril que es voldra seguir per defecte)
	int current_num_lanes=Lanes.size(); // number of lanes of the track
	int desired_lane=0; // desired lane where we want to drive (Right lane is 0, center lane is 1, left lane is 2 ...) 
	
	
	vector <vector<float> > GP_path;
	
	GP_path.push_back(Highway.center_paths[desired_lane].traj[0]); //incloc les Xs
	GP_path.push_back(Highway.center_paths[desired_lane].traj[1]); //incloc les Ys
	GP_path.push_back(Highway.center_paths[desired_lane].traj[2]); //incloc les curvatures
	
	int lane_change = 0; // 0 means follow lane, +1 left lane change, -1 right lane change --> during the routine it will change
	
	float max_road_vel = 14; // max velocity allowed on the road ( 14 m/s --> 50 Km/h) [m/s]
	
	
	//---------------------------//
	//Obstacles Estatics
	C_StaticObs static_obs (GP_path); // declaracio de la variable 'static_obs' que representara el conjunt de obstacles de la carretera
									  // Aplicacio de la clase de C_StaticObs per a representar el conjunt d'obstacle de la carretera

	//Definicio obstacle 1
	float coord_obstacle[2];
	coord_obstacle[0]=110;//110;
	coord_obstacle[1]=-3;  
	float radi_obstacle=3; 
	int id_obstacle=1;	   
	
	C_Obstacle obstacle1 (coord_obstacle, radi_obstacle, id_obstacle); // creem obstacle amb les variables donades
	static_obs.add_obs(obstacle1); // l'afegim al conjunt de obstacles associats a la carrtera
	
	
	//---------------------------//
	// Initial conditions
	float vel=14; // velocitat longitudinal que te el cotxe
	
	int current_lane=0; //indiquem el carril actual en que ens trobem
	pos_X= Highway.center_paths[current_lane].traj[0][0]; 
	pos_Y= Highway.center_paths[current_lane].traj[1][0];
	float pos[2] ={pos_X, pos_Y}; // posicio inicial del cotxe
	
	heading=0; //orientacio inicial del cotxe en graus (>0 cap a l'esquerra) [graus]
	float delta = 0; //orientnacio inicial de la direccio en graus (>0 girar cap a l'esquerra) [graus]
	float delta_meas= delta;
	float last_delta_meas= delta;
	float curvature = tan(delta*M_PI/180)/car_wheelbase;
	float yawr=0; // not used ...
	float delta_state = delta; // not used in the algorithm // state initialization for the first order transfer function
	
	float accel= 0; // Current acceleration. in m/s^2
	float accel_last = accel; // accel of the last instant
	
	float pos_X_fw = pos_X + car_wheelbase*cos(heading*M_PI/180); //coordenades en X i Y de la part davantera del cotxe
	float pos_Y_fw = pos_Y + car_wheelbase*sin(heading*M_PI/180);
	
	//---------------------------//
	// Simulacio
	float total_T=100; //temps total de la execucio de l'algoritme (cal ?¿?)
	float delta_t=0.02; //0.02 [segons] interval de discretitzacio de temps
	
	vector <vector<float> > last_traj = GP_path; // ultima trajectoria enviada al controlador (vector de coord X's, vector de coord Y's i curvatures) // el de les condicions inicials , sera el carril a seguir per defecte
	
	int new_controller_update = 1; // 1 means that a new obstacle has been detected
	float index_simu = 1;
	
	//---------------------------//
	// Inicialitzacio variables representaran les dades de la iteracio anterior
	float last_vp_finalvel = 1000; // velocitat final del ultim perfil de velocitat enviat
	float last_vp_accel = 1000; // acceleracio final del ultim perfil d'acceleracions enviat
	float last_time = -10; // last time of a new velocity profile sent
	
	vector <float > temps;
	temps.push_back(1000); // vector de temps
	
	float last_target_i_GP = 0; // ultima 'station' (numero de coordenada del carril) objectiva del cami a seguir 
	vector <float > last_target_i_highway; //  % Variable to store where are we now and not start counting from the beginning --> vector que indica el numero de punt 'station' que estem de cada carril de la carretera --> al inici tot 0s
	for (int i=0; i<Highway.num_lanes; i=i+1)
	{
		last_target_i_highway.push_back(0); // les posem totes a la 'station' inicial (a 0)
	}
	
	//Creem un objecte de la estructura 'C_mandatory_point' el qual representa un punt critic del circuit, on s'haura de realitzat una certa maniobra ( salvar obstacle) ¿¿??¿?
	C_mandatory_point mandatory_point; // inicialitza un dels membres de la classe C_SamplePoint, tot a 0's
	mandatory_point.GP_path_station=0;
	mandatory_point.reason=0; //  1 -> changing lane, 2 -> static obstacle avoidance, 3 -> static obstacle avoidance
	mandatory_point.point.s=-1; // estacio per decfecte a -1
	
	int reset_manoeuvre_flag = 0; // Is for resetting a manoeuvre when we stop due to dynamic obstacles
	
	
	//---------------------------//
	// Dynamic obstacles
	
	
	//***(CREACIO VEHICLE CIRCULA PEL NOSTRA CARRIL --> FORMA PER TRES CERCLES)*** '1i'
	
	// ----- Obstacle following the same lane as us (circula mateix sentit que nosaltres) ------
	float path_dist_1 = 150; //distancia que ha de recorrer el vehicle
	float dyn_veh_len = 5; //longitud de llargada que fa el vehicle
	float r_veh=0.6; //radi de circunferencia que representa al vehicle
	
	float dyn_veh_len_ahead = 10; // m ?¿?¿?¿
	//float dyn_veh_plot_station = round(3.0/resolution); // ¿?¿?¿
	
	float dyn_veh_vel=5; //velocitat a la que es mou el vehicle (m/s)
	float dyn_veh_ini_t=0.2; //temps en que es comença a moure el vehicle
	
	float dist_lane_ini_veh=40.0; //distancia en metres inicial a la que comença el vehicle
	float initial_index = round(dist_lane_ini_veh/resolution)-1; //el indice inicial sera el correspondiente a 40 metros (restem 1, ja que comencem amb el index 0) ([m]/resolution) 
	
	// Construccio cami que seguira el vehícle (format per un vector de les coordenades X i un altre amb les coordenades Y)
	vector<float> Px1;
	vector<float> Py1;
	vector <vector<float> > path_veh; // vector de 2 vectors que representa les coordenades del cami a seguir pel vehicle , es construiran a partir dels camins Px1 i Py1
	
	for (int i=initial_index; i<=(initial_index+round(path_dist_1/resolution)); i=i+1)
	{
		Px1.push_back(Highway.center_paths[0].traj[0][i]);
		Py1.push_back(Highway.center_paths[0].traj[1][i]);
	}
		
	int lane_veh=0; // numero de carril en que es troba el vehicle ( el de mes a la dreta)
	int id=1; // numero identificador del vehicle
	
	
	
	vector<C_DynObstacle> dyn_obs; //vector que contindra element de la clase C_DynObstacle
	vector<C_DynObstacle> dyn_obs_plot; // un altre
	
	// Creacio dels diferents obstacles mobil
	
	//CERCLE 1 (part trasera cotxe)
	// vectors que representen la trajectoria que seguira l'obstacle mobil
	vector<float> Px11;
	vector<float> Py11;
	
	// Creacio trajecroria que seguira el 1er obstacle mobil
	for (int i=0; i<(Px1.size()-round(dyn_veh_len/resolution)); i=i+1) // treiem els punts corresponents a la longitud del cotxe
	{
		Px11.push_back(Px1[i]);
		Py11.push_back(Py1[i]);
	}
	
	path_veh.push_back(Px11);
	path_veh.push_back(Py11);
	
	C_DynObstacle dyn_obs1 ( dyn_veh_ini_t, delta_t, dyn_veh_vel, r_veh, path_veh, path_dist_1-dyn_veh_len, resolution, id, lane_veh) ;
	
	path_veh.clear();
	
	//CERCLE 2 (part davantera cotxe)
	// vectors que representen la trajectoria que seguira l'obstacle mobil
	vector<float> Px12;
	vector<float> Py12;
	
	// Creacio trajecroria que seguira el 1er obstacle mobil
	for (int i=round(dyn_veh_len/resolution)-1; i<(int)Px1.size(); i=i+1) // treiem els punts corresponents a la longitud del cotxe
	{
		Px12.push_back(Px1[i]);
		Py12.push_back(Py1[i]);
	}
	
	path_veh.push_back(Px12);
	path_veh.push_back(Py12);
	
	C_DynObstacle dyn_obs2 ( dyn_veh_ini_t, delta_t, dyn_veh_vel, r_veh, path_veh, path_dist_1-dyn_veh_len, resolution, id, lane_veh) ;
	
	path_veh.clear();
	
	//CERCLE 3 (part visible del cotxe)
	// vectors que representen la trajectoria que seguira l'obstacle mobil
	vector<float> Px13;
	vector<float> Py13;
	
	// Creacio trajecroria que seguira el 1er obstacle mobil
	for (int i=round(dyn_veh_len_ahead/resolution)-1; i<Px1.size(); i=i+1) // treiem els punts corresponents a la longitud del cotxe
	{
		Px13.push_back(Px1[i]);
		Py13.push_back(Py1[i]);
	}
	
	path_veh.push_back(Px13);
	path_veh.push_back(Py13);
	
	C_DynObstacle dyn_obs3 ( dyn_veh_ini_t, delta_t, dyn_veh_vel, r_veh, path_veh, path_dist_1-dyn_veh_len_ahead, resolution, id, lane_veh) ;
	
	path_veh.clear();
	
	
	// Afegim els obstacles dinamics al vector dyn_obs
	dyn_obs.push_back(dyn_obs1); 
	dyn_obs.push_back(dyn_obs2); 
	dyn_obs.push_back(dyn_obs3); 
	
	float num_dyn_obs= dyn_obs.size();
	// proves obstacles dinamics 
	/*
	cout << dyn_obs.size() << endl;
	
	for (int i=0; i<dyn_obs[2].path[0].size(); i=i+1) // treiem els punts corresponents a la longitud del cotxe
	{
		cout << dyn_obs[2].path[0][i] << endl ;
	}
	*/
	
	// ----- Obstacle following another lane as us , in inverse way (circula sentit contrari que nosaltres, en un altre carril) ------
	
	
	
	int dyn_obs_avoidance = 0; //% Flag for performing an avoidance manoeuvre. // 1 means we can avoid dinamic obstacles
	
	// FALTA DEFINIR AQUESTS TIPUS DE OBSTACLES
	
	
	
	//---------------------------//
	// ROUTINE OF THE ALGORITHM 

	
	
	for (float t=0; t<0.1; t=t+1) // (cas 1 cop) (float t=0; t<=total_T; t=t+delta_t) //--> el bo
	{ //ACTIVAR 'FOR' QUAN ACAVEM 
		
		/*
					if (t == 5.00)
					{
						dyn_obs_avoidance = 1; //% We want to avoid a dynamic obstacle now!
				
					}
		*/
		
		//float t=0; // TREURE EN POSAR EL LOOP
		
		//Donada la posicio del cotxe actual, calculo el 'numero de punt' més proper del carril que vull seguir i la distancia fins ell. Tenin present el 'numero de punt' en que ens trobavem a la iteracio anterior
		float target_i_GP;
		float dist_to_GP;
		
		Search_closest_point(pos_X, pos_Y, GP_path[0], GP_path[1], last_target_i_GP, target_i_GP, dist_to_GP);
		
		// Calcul del tram del carril que els sensors em permeten captar
		float initial_lookahead_dist=100; // metres que visualitzem per davant (PARAMETRE)
		float lookahead_dist=initial_lookahead_dist+resolution;
		float lookahead_points = ceil(lookahead_dist/resolution); // calcul del 'numero de punts' de carretera que visualitzem i respecte els quals actuarem 
		
			
		
		//Calcul possibilitats de canvi de carril que tinc
		int target_lanes[3]={current_lane+1, current_lane, current_lane-1}; // {index carril de l'esquerra, indexs del nostre carril , index carril de la dreta}
		int possible_lanes[3]; // ens indica amb un 1 (si podem anar a aquell carril o ja estem en ell) o amb un 0, si no podem
		
		for (int i=0; i<3; i=i+1)
		{
			if ( (target_lanes[i]>=0) && (target_lanes[i]<=(current_num_lanes-1)) )
			{
				possible_lanes[i]=1; // 1 indica que es podra canviar a aquell carril o estem en aquell carril (el carril actual es troba al costat d'aquell carril o es ell mateix )
			}
			else
			{
				possible_lanes[i]=0; // 0 indica que no ens podem canviar a aquell carril
			}
		}
		
				
		//Creacio de LP 'LookaheadPath' --> Conjunt de punts de cadascun dels carrils als que puc anar (i el propi en el que estic) i els quals son detectats pels sensors del cotxe

		vector <vector<vector<float> > > LP_paths; // [ carril_esquerra[ [coordX[], coordY[], curvR[] ]  ,  carril_actual[...]  ,  carril_dreta[...] ]
		
		for (int i=0; i<3; i=i+1) //per a cada carril que tenim al costat y en el propi que estem
		{
			if (possible_lanes[i]==1) //si el carril es accessible
			{
				float lane= target_lanes[i];
			
				float target_i_LP;
				float dist_to_LP;
		
				Search_closest_point(pos_X, pos_Y, Highway.center_paths[lane].traj[0], Highway.center_paths[lane].traj[1], last_target_i_highway[lane], target_i_LP, dist_to_LP); // busquem el 'número de punt' d'aquell carril que esta més aprop del nostre cotxe
				
				if ( (target_i_LP+lookahead_points) <= Highway.num_points[lane] ) // cas en que els punts que podem tractar, son menors als punts que li queden al carril (agafem una porcio del total de punts del carril, que van del punt mes proper del carril cap el nostre cotxe, fins a aquell punt que ens permeti el nostre horitzo de visualitzacio)
				{
					vector <vector<float> > LP_lane_aux;
					vector<float> lane_Xelements_aux;
					vector<float> lane_Yelements_aux;
					vector<float> lane_Relements_aux;
					
					for (int i=target_i_LP; i<=(target_i_LP+lookahead_points); i=i+1)
					{
						lane_Xelements_aux.push_back(Highway.center_paths[lane].traj[0][i]); //introduim les coordenades X que tractarem
						lane_Yelements_aux.push_back(Highway.center_paths[lane].traj[1][i]); //introduim les coordenades Y que tractarem
						lane_Relements_aux.push_back(Highway.center_paths[lane].traj[2][i]); //introduim les curvatures r que tractarem
					}
					
					LP_lane_aux.push_back(lane_Xelements_aux);
					LP_lane_aux.push_back(lane_Yelements_aux);
					LP_lane_aux.push_back(lane_Relements_aux);

					LP_paths.push_back(LP_lane_aux); //introduim el carril amb els punts als que podem accedir en el LP (LookaheadPath)
					
					//fem neteja variables em emprat per construir LP_paths 
					lane_Xelements_aux.clear();
					lane_Yelements_aux.clear();
					lane_Relements_aux.clear();
					LP_lane_aux.clear();
				}
				
				else // cas en que queda menys de distancia de carretera, que la nostra capacitat de visualitzacio
				{
					vector <vector<float> > LP_lane_aux;
					vector<float> lane_Xelements_aux;
					vector<float> lane_Yelements_aux;
					vector<float> lane_Relements_aux;
					
					for (int i=target_i_LP; i<=(int)(Highway.center_paths[lane].traj[0].size()); i=i+1)
					{
						lane_Xelements_aux.push_back(Highway.center_paths[lane].traj[0][i]); //introduim les coordenades X que tractarem
						lane_Yelements_aux.push_back(Highway.center_paths[lane].traj[1][i]); //introduim les coordenades Y que tractarem
						lane_Relements_aux.push_back(Highway.center_paths[lane].traj[2][i]); //introduim les curvatures r que tractarem
					}
					
					LP_lane_aux.push_back(lane_Xelements_aux);
					LP_lane_aux.push_back(lane_Yelements_aux);
					LP_lane_aux.push_back(lane_Relements_aux);
					
					LP_paths.push_back(LP_lane_aux); //introduim el carril amb els punts als que podem accedir en el LP (LookaheadPath)

					//fem neteja variables em emprat per construir LP_paths 
					lane_Xelements_aux.clear();
					lane_Yelements_aux.clear();
					lane_Relements_aux.clear();
					LP_lane_aux.clear();
				}
			} 
			
			else // omplim amb -1
			{
				vector<float> lane_elements_aux;
				vector <vector<float> > LP_lane_aux;
				lane_elements_aux.push_back(-1); 
				LP_lane_aux.push_back(lane_elements_aux);
				LP_paths.push_back(LP_lane_aux); //introduim un '-1' en el carril que no podem accedir
				
				lane_elements_aux.clear();
				LP_lane_aux.clear();			
			}
		}
		
		//Valors parametres acceleracions
		float max_braking_accel = 7; // In abs value /maxima aceleracion de freno (PARAMETRE)
		float comfort_a_lat = 3; // maxima aceleracion lateral ?¿?¿? (PARAMETRE)
		float comfort_a_long = 1.5; // maxima aceleracion logitudinal ?¿?¿ (PARAMETRE)
		
		//Calcul de les maximes velocitats permeses en el tram (depen de la curvatura del tram)
		float max_vel_restriction [2]; //array ; 1er element --> Distancia longitudinal [m] a apartir de la qual s'aplica la restriccio de velocitat (abans s'aplica la propia de al carretera)
									        //	 2on elemetn --> Velocitat maxima admesa a partir de la distancia indicada pel primer element [m/s]
		max_admissible_vels (LP_paths[1][2], resolution, comfort_a_lat, max_road_vel, max_vel_restriction);
		
				
		//Calcul del 'heading' (orientacio) del vehicle --> element que falta per definir el estat X del vehicle --> [x, y, headind, curvature]
		float head_traj; // orientacio del vehicle en graus respecte la hoitzontal (degrees)
		//Busquem el 'numero de punt' mes proper al carril que teniem a la ultima iteracio respecte a la posicio actual 
		float target_j;
		float dist_j;
		Search_closest_point (pos_X, pos_Y, last_traj[0], last_traj[1], 0, target_j, dist_j);
		
		// Calculem distancia del 'numero de punt' del carril mes proxim, fins el 'número de punt' seguent  
		float d = sqrt( pow(( last_traj[0][target_j] - last_traj[0][target_j+1] ),2) + pow( (last_traj[1][target_j] - last_traj[1][target_j+1]),2) );
				
		//calcul de la orientacio del vehicle 'heading' (angle respecte la horitzontal)
		float d_min=0.001; // distacia minima per a suposar que estem en un numero de punt de la carretera diferent
		if (d>=d_min)
		{
			head_traj= constrain_angle( atan2( (last_traj[1][target_j+1]-last_traj[1][target_j]) , (last_traj[0][target_j+1]-last_traj[0][target_j]) )*(180/M_PI) , 360.0 ); // atan2(coordY, coordX)
		}
		else // cas en que la distancia es tan petita que ve a signifiacar que ens trobem en el mateix 'numero de punt' --> agafem el seguent numero de punt ( 2 mes endavent)
		{
			head_traj= constrain_angle( atan2( (last_traj[1][target_j+2]-last_traj[1][target_j]) , (last_traj[0][target_j+2]-last_traj[0][target_j]) )*(180/M_PI) , 360.0 );
		}
				
					
		// ?¿?¿?¿?
		if (vel >= -2.9) 
		{
			reset_manoeuvre_flag = 0; // % Is for resetting a manoeuvre when we stop due to dynamic obstacles
		}
		
		
		float min_index=10; // (PARAMETRE)  ?¿?¿?minim punt en que ens trobem ?¿?¿, a partir del qual si ens apropem mes ja no considerem el punt critic ?¿?¿ 
		// cas en que ens trobem a 10 'numeros de punts o mes alluñats respecte una 'station' (punt critic)
		
		//inicialitzacio varibles global
		C_SamplePoint initial_point; //s'inicialitza amb el constructor per defecte (variable global)
		vector <vector<float> > LP_path; // carril unic
		float target_i_GP_;
		C_Traj best_traj; // millor trajectoria que sera escollida per a ser executada
		
		if ((mandatory_point.GP_path_station-min_index) > target_i_GP) // If we must follow a mandatory manoeuvre. With a little offset for security ?¿?¿?
		{
			initial_point=mandatory_point.point;
			C_Traj best_traj=mandatory_point.traj;
			target_i_GP_=mandatory_point.GP_path_station;
			
			// calcul del 'numero de punt' mes proper del carril on anem
			float target_k;
			float dist_k;
			Search_closest_point ( initial_point.x, initial_point.y, LP_paths[1][0], LP_paths[1][1], 0, target_k, dist_k);
			
			if (lookahead_points - target_k < min_index) // volem anar a un 'numerod de punt' que es un dels mes allunayts que podem visualitzar
			{
				target_k = lookahead_points - min_index; // fins el final de punts a considerar respecte el limit, considerarem sempre la mateixa estacio
			}
			
			// % target_k is the station used by the mandatory traj
			vector <float > path_x_aux;
			vector <float > path_y_aux;
			vector <float > path_k_aux;
			for (float i=target_k; i<=LP_paths[1][0].size(); i=i+1)
			{
				path_x_aux.push_back(LP_paths[1][0][i]); // afegeixo x
				path_y_aux.push_back(LP_paths[1][1][i]); // afegeixo y 
				path_k_aux.push_back(LP_paths[1][2][i]); // afegeixo curvatura
			}
			//construeixo el carril
			LP_path.push_back(path_x_aux);
			LP_path.push_back(path_y_aux);
			LP_path.push_back(path_k_aux);
			
			path_x_aux.clear();
			path_y_aux.clear();
			path_k_aux.clear();
			
			// resetegem maniobra si anem a parar
			float vel_min=0.1; // velocitat minima que indica que pararem (m/s)
			if ( (vel < vel_min) && (reset_manoeuvre_flag == 0) )
			{
				// resetegem el objecte 'mandatory point'
				mandatory_point.GP_path_station = 0;
				mandatory_point.reason = 0;
				C_Traj aux; //to clean trajectory
				mandatory_point.traj=aux;
				// indiquem que em parat per salvar un obstacle
				reset_manoeuvre_flag = 1; 				
			}
		}
		else // % Normal conditions, recomputing always the path from our position
		{
			mandatory_point.GP_path_station = 0; // % Resetting (it is not mandatory)
			mandatory_point.reason = 0; // ens quedem al mateix carril //  1 -> changing lane, 2 -> static obstacle avoidance, 3 -> static obstacle avoidance
			C_Traj aux; //to clean trajectory
			mandatory_point.traj=aux;
						
			//A partir de les coordenades del ultim punt per on em passat i la orientacio del vehicle calculem el seu 'initial state' --> coordenades del cotxe, curvatura y orientacio del ultim punt on es trobava el cotxe
			C_SamplePoint initial_point2 (last_traj[0][target_j], last_traj[1][target_j], last_traj[2][target_j], head_traj,-1); //te da las coordenadas, la curvatura y la orientacion del ultimo estado del vehiculo
			initial_point=initial_point2;
			//vector <vector<float> > best_traj; // millor trajectoria que sera escollida per a ser executada
			target_i_GP_ =  target_i_GP; // index del carril objectiu
			
			LP_path=LP_paths[1]; // carril actual en que ens trobem
		}
				
		// Discretitzacio de l'espai
		// % Lattice longitudinal distances --> (station factors) --> discretitzacio en forma longitudinal al carril
		float m[] = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1};
		vector<float> dists(m, m + sizeof(m) / sizeof (*m) );
		// % Lattice lateral distances (<0 is to the right of the lane) --> (lateral offsets) --> discretizacion en forma perpendicular al carril
		float n[] = {0.5, 0.4, 0.3, 0.15, 0, -0.15, -0.3, -0.4, -0.5};
		vector<float> offsets(n, n + sizeof(n) / sizeof (*n) );
		// paths are generated from the current host vehicle state to each set of different endpoints situated in the same station, but with different lateral offsets
		
		// Distancia entre el 'initial point' (on es troba el cotxe) i un 'endpoint' de la generacio d'un 'path' --> disntcia d'interpolacio
		float security_distance=20; // distancia de seguretat per asegurar la frenada total del vehicle 
		float sampling_distance; // distancia de mostreig que emprarem en metres --> cada aquesta distancia inclourem una 'station')
		sampling_distance=sampling_dist (vel, max_braking_accel, security_distance); 
				
		
		// Discretitzacio espaial del tram de carril 'LP_path' a tractar 
		C_Sampling sampling_ahead (LP_path, resolution, sampling_distance, dists, offsets, initial_point, target_i_GP_, static_obs);
		
		S_lane_sampling lane_sampling; //varible pertanyen a l'estructura S_lane_sampling que conte total la inforamcio de la discretitzacio del tram del carril
		sampling_ahead.sample_lane (car_3circle_r, lane_sampling);
		
		vector <vector<C_SamplePoint> > traj_points; // matriu amb la discretitzacio espaial i la informacio de cadascun dels punts en que ha estat feta la discretització
		vector <vector<int> > feasible_traj_points; // matriu de 1s i/o 0s --> 1 indica que no hi ha cap obstacle pertorbant aquell punt, 0 es que hi ha obstacle
		vector<float> traj_stations; // numeros de punts (indexos del carril) on es troba cada una de les 'stations' amb que discretitzem el carril ( cada columna de 'traj_points' compartira la mateixa 'station' )
		vector<int> obstruction_here; // indica amb 0 la 'station' on no hi ha cap obstacle, i amb un 1 on hi ha obstacle (encara que sigui nomes en un dels seus punts de mostreig(offset))
		float max_feasible_station; // valor del index de la 'station' mes allunyada
		int lane_blocked; // indica amb un 1 que no es pot passar pel carril (totalment bloquejat) i amb un 0 de que si es pot pasar ( tot i que por tenir obstacles)
		float first_sampling_index; // index del vector de 'traj_stations' que indica la 'station' on s'acaba la discretitzacio 'fina' --> provocat pel vector de 'dists'
		
		traj_points = lane_sampling.traj_points;
        feasible_traj_points = lane_sampling.feasible_traj_points;
        traj_stations = lane_sampling.traj_stations;
        obstruction_here = lane_sampling.obstruction_here;
        max_feasible_station = lane_sampling.max_feasible_station;
        lane_blocked = lane_sampling.lane_blocked;
		
		// Construim vector que indica quines 'stations' han de seguir el mateix offset de la 'station' previa degut a la presencia de obstacles --> 'offset_combinations'
		
		S_lane_sampling_comb new_lane_sampling;
		
		sampling_ahead.traj_combinations (lane_sampling, new_lane_sampling);
		
		vector <vector<int> > new_feasible_traj_points; //variables noves
		vector<float> offset_combinations; 
		
		traj_points = new_lane_sampling.traj_points;
        new_feasible_traj_points = new_lane_sampling.feasible_traj_points;
        traj_stations = new_lane_sampling.traj_stations;
        obstruction_here = new_lane_sampling.obstruction_here;
        max_feasible_station = new_lane_sampling.max_feasible_station;
        lane_blocked = new_lane_sampling.lane_blocked;
        first_sampling_index = new_lane_sampling.first_sampling_index;
        offset_combinations = new_lane_sampling.offset_combinations;
		
				
		// Calcul de les trajectories candidates a executar
		
		C_TrajGen new_traj (new_lane_sampling, initial_point, target_i_GP_, resolution, static_obs);
				
		// Generem les trajectories fins a la primera 'station' del mostreig espaial fet
		vector<C_Traj> initial_trajs;// vector amb elements C_Traj que contindra les trajectories candidates
		new_traj.generate_initial_opt (car_3circle_r, min_turn_r, initial_trajs); 
		
				
		// Generem les trajectories desde la 'station' ultima del mostreig esaial fi fins les 'stations' del mostreig espaial ampliat
		vector<C_Traj> rest_trajs;
		new_traj.generate_rest (rest_trajs); 
		
		// Unim les trajectories (inicials amb la resta)
		vector<C_Traj> trajs; // trajectories totals generades (candidates a ser escollides)
		vector<float> c_lengths; // vector de costos de longitud asociats a les trajectories
		vector<float> c_maxks; // vector de costos de les curvatures asociats a les trajectories
		vector<float> c_maxkdots; // vector de costos de derivades de curvatures asociats a les trajectories
		vector<float> c_offsets; // te els indexs dels offsets // ?¿?¿vector de costos pels offsets (contra mes allunyat del carril central mes penalitat) asociats a les trajectories
		vector<float> c_static_obs; // vector de costos pels obstacles presents asociats a les trajectories
		vector<float> collisions; // indica els indexs de les 'stations' on es produeixen colisions // si no es produeix colisio, s'indica amb un -1
		
		new_traj.merge (initial_trajs, rest_trajs, car_3circle_r, trajs, c_lengths, c_maxks, c_maxkdots, c_offsets, c_static_obs, collisions);

		// Eleccio de la trajectoria optima ( la de menys cost associat )
		C_Traj best_traj_2; // millor trajectoria
		float best_traj_2_cost; // cost asociat a la millor trajectoria
		float best_traj_2_col; // per indicar en quina station hi ha colisio per a aquella trajectoria (-1) si no hi ha
		float best_traj_i; // index del vector de trajectories que correspon a la trajectoria escollida
		
		new_traj.best_path (trajs, c_lengths, c_maxks, c_maxkdots, c_offsets, c_static_obs, collisions, min_turn_r, best_traj_2, best_traj_2_cost, best_traj_2_col, best_traj_i);


		//---
		
		float best_traj_col;
		float target_j_best_traj; //?¿?¿?
			
		if ( !(best_traj.len==0) ) // cas em que tenim trajectoria 'mandatory' (obligatoria) (la longitud NO es 0) --> % Need to follow a mandatory traj (necesitat de seguir una trajectoria obligatoriament abans de executar la trajectoria escollida)
		{
			cout << 5 << endl ;
			
			float ini_point_sampling_coords[2];
			ini_point_sampling_coords[0]=-1;
			ini_point_sampling_coords[1]=-1;
			
			float end_point_sampling_coords[2];
			end_point_sampling_coords[0]=-1;
			end_point_sampling_coords[1]=-1;
			
			vector <vector<float> > traj_tot;
			traj_tot=best_traj.traj;
			
			for (int i=0; i<(int)best_traj_2.traj[0].size(); i++) 
			{
				traj_tot[0].push_back(best_traj_2.traj[0][i]);
				traj_tot[1].push_back(best_traj_2.traj[1][i]);
				traj_tot[2].push_back(best_traj_2.traj[2][i]);
			}
			
			float dist_tot=best_traj.len + best_traj_2.len;
			
			C_Traj best_traj (ini_point_sampling_coords, end_point_sampling_coords, traj_tot, dist_tot, resolution);
			
			if (best_traj_2_col > -1) // hi ha colisio en alguna 'station'
			{
				best_traj_col = (mandatory_point.GP_path_station - target_i_GP) + best_traj_2_col; 
				
			}
			else
			{
				best_traj_col = best_traj_2_col;
			}
			
			float target_j_best_traj;
			float dist; //no s'usa
			
			
			
			Search_closest_point(pos_X, pos_Y, best_traj.traj[0], best_traj.traj[1], 0, target_j_best_traj, dist); // millora de l'anterior
        }
        else // No need to follow mandatory traj / (no cal seguir una trajectoria obligatoria determinada)
        {
			best_traj = best_traj_2; // la millor trajectoria es la ja trobada
			best_traj_col = best_traj_2_col;
			target_j_best_traj = 0; //?¿?¿?
	    }
	   	 
	   
	   ////pruebas // HERE 3/11/2018
		
		
		/*
		for (int i=0; i<best_traj.traj[0].size(); i++)
		{
			cout << best_traj.traj[1][i] << endl ;
		}
		
		cout << best_traj_2_cost << endl ;
		cout << best_traj_i << endl ;
	   
	   
	   */
	   
	   
	   
	    // Calcul restriccions de velocitat i llocs on es produeixen colisions 
	    vector <vector<float> > vel_restrictions;
	   	// vel_restr -->  vel_restr[0] = [max_k_dist , max_vel_admissible] --> [distancia longitudinal a la que es produeix la maxim curvatura (metres) , maxima velocitat admisible degut a la curvatura mes pronunciada existent]
		//				  vel_restr[1] = [max_col_dist , max_allowed_vel_by_col_or_k] --> [distancia minima on es produeix colisio (per curvatura o obstacle , maxima velocitat quan hi ha colisio o curvatura]	
	    new_traj.compute_vel_static_restrictions (best_traj, target_j_best_traj, best_traj_col, min_turn_r, comfort_a_lat, vel_restrictions);

	    // Calcul del millor perfil de velocitat
	    vector<vector<float> > best_vp; // perfil de velocitats millor (X's, V's, A's, t's)
	    float best_vp_min_cost;
	    float vp_accel;
	    float vp_finalvel;
	    
	    new_traj.compute_best_vp ( best_traj, target_j_best_traj, vel_restrictions, max_vel_restriction, vel, max_road_vel, accel, accel_last, -max_braking_accel, comfort_a_long, t, delta_t, dyn_obs, car_1circle_r, lookahead_dist, best_vp, best_vp_min_cost, vp_accel, vp_finalvel);

		//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

	    if ( (best_traj_col>-1) && (mandatory_point.reason==0) ) //  % Only if we are not in other manoeuvres
	    {

			vector <vector<C_SamplePoint> > traj_points_L; 
		    vector <vector<int> > feasible_traj_points_L; 
		    vector<float> traj_stations_L; 
		    vector<int> obstruction_here_L; 
		    float max_feasible_station_L; 
		    int lane_blocked_L; 
		   
		   
		    if (possible_lanes[0]==1) //% Left lane change possible / podem canviar al carril esquerre
		    {
			   //% LEFT LANE SAMPLING**** --> mostregem el carril esquerra --> LP_paths[0]
				C_Sampling sampling_L ( LP_paths[0], resolution, sampling_distance, dists, offsets, initial_point, target_i_GP_, static_obs);
			   
				S_lane_sampling lane_sampling_L;
				int lane_change_L_finished;
				sampling_L.avoidance_other_lane (car_3circle_r, LP_paths[1], best_traj_col, lane_sampling_L, lane_change_L_finished); 
				
				traj_points_L = lane_sampling_L.traj_points;
				feasible_traj_points_L = lane_sampling_L.feasible_traj_points;
				traj_stations_L = lane_sampling_L.traj_stations;
				obstruction_here_L = lane_sampling_L.obstruction_here;
				max_feasible_station_L = lane_sampling_L.max_feasible_station;
				lane_blocked_L = lane_sampling_L.lane_blocked;
			   
				if (lane_change_L_finished == 1) // ja s'ha realitzat la maniobra de canvi al carril esquerra
				{
					//vector <vector<C_SamplePoint> > traj_points_L; 
					vector <vector<int> > new_feasible_traj_points_L; 
					//vector<float> traj_stations_L; 
					//vector<int> obstruction_here_L; 
					//float max_feasible_station_L; 
					int lane_blocked_L; 
					float first_sampling_index_L;
					vector<float> offset_combinations_L;
					
					S_lane_sampling_comb new_lane_sampling_L;
					sampling_L.traj_combinations(lane_sampling_L, new_lane_sampling_L);
					
					traj_points_L = new_lane_sampling_L.traj_points;
                    new_feasible_traj_points_L = new_lane_sampling_L.feasible_traj_points;
                    traj_stations_L = new_lane_sampling_L.traj_stations;
                    obstruction_here_L = new_lane_sampling_L.obstruction_here;
                    max_feasible_station_L = new_lane_sampling_L.max_feasible_station;
                    lane_blocked_L = new_lane_sampling_L.lane_blocked;
                    first_sampling_index_L = new_lane_sampling_L.first_sampling_index;
                    offset_combinations_L = new_lane_sampling_L.offset_combinations;


					C_TrajGen new_traj_L (new_lane_sampling_L, initial_point, target_i_GP_, resolution, static_obs);
					
					vector<C_Traj> initial_trajs_L;
					new_traj_L.generate_initial_opt( car_3circle_r, min_turn_r, initial_trajs_L); 

					vector<C_Traj> rest_trajs_L;
					new_traj_L.generate_rest (rest_trajs_L); 
					
					
					vector<C_Traj> trajs_L;
					vector<float> c_lengths_L;
					vector<float> c_maxks_L;
					vector<float> c_maxkdots_L;
					vector<float> c_offsets_L;
					vector<float> c_static_obs_L;
					vector<float> collision_L;
					new_traj_L.merge(initial_trajs_L, rest_trajs_L, car_3circle_r, trajs_L, c_lengths_L, c_maxks_L, c_maxkdots_L, c_offsets_L, c_static_obs_L, collision_L);

					C_Traj best_traj_L;
					float best_traj_min_cost_L;
					float best_traj_col_L;
					float best_traj_i_L;
					new_traj_L.best_path (trajs_L, c_lengths_L, c_maxks_L, c_maxkdots_L, c_offsets_L, c_static_obs_L, collision_L, min_turn_r, best_traj_L, best_traj_min_cost_L, best_traj_col_L, best_traj_i_L);
					
					if ( (1/best_traj_L.maxk) <= min_turn_r ) // no hi ha colisio per curvatura
					{
						best_traj_col_L = 1;
					}
					
					float final_point_coords[2]; //[station] i [offset]
					
					final_point_coords[0] = best_traj_L.end_point_sampling_coords[0]; // station
					final_point_coords[1] = best_traj_L.end_point_sampling_coords[1]; // offset
					
					C_SamplePoint final_point; 
					final_point=traj_points_L[final_point_coords[0]][final_point_coords[1]];
					
					float avoidance_pos[2];
					avoidance_pos[0]=LP_paths[1][0][best_traj_col]; //coord X
					avoidance_pos[1]=LP_paths[1][1][best_traj_col]; //coord Y
					
					float av_i_GP;
					float dist_av; // no se usa
					
					Search_closest_point( avoidance_pos[0], avoidance_pos[1], GP_path[0], GP_path[1], last_target_i_GP, av_i_GP, dist_av); // millora de l'anterior

					//.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*
					C_LaneChange new_lane_change ( LP_paths[1], LP_paths[0], resolution, sampling_distance, dists, offsets, initial_point, target_i_GP, static_obs, car_3circle_r, min_turn_r);
					
					// mirem si l'altre carril esta bloquejat (1--> estaq bloquejat)
					int other_lane_blocked=new_lane_change.lane_change_is_blocked(); //
					
					if ( (other_lane_blocked == 0) && (best_traj_col_L==-1) ) // no esta l'altre carril bloquejat i cap index indica colisio
					{
						vector<C_Traj> trajs_av;
						vector<float> c_lengths_av;
						vector<float> c_maxks_av;
						vector<float> c_maxkdots_av;
						vector<float> c_static_obs_av;
						vector<float> collisions_av;
						int lane_change_possible;
						new_lane_change.gen_avoidance ( 8, best_traj, best_traj_L, avoidance_pos, trajs_av, c_lengths_av, c_maxks_av, c_maxkdots, c_static_obs_av, collisions_av, lane_change_possible);  
						
						if ( lane_change_possible == 1 ) // compute best trajectory (with the best veocity profile)
						{
							C_Traj best_traj_ch;
							float best_traj_cost_ch;
							float best_traj_col_ch;
							vector<vector<float> > best_traj_vp_ch;
							float best_traj_vp_cost_ch;
							float best_vp_accel_ch; 
							float best_vp_finalvel_ch;
							new_lane_change.best_lane_change_traj ( trajs_av, c_lengths_av, c_maxks_av, c_maxkdots_av, c_static_obs_av, collisions_av, comfort_a_lat, max_vel_restriction, vel, max_road_vel, accel, accel_last, -max_braking_accel, comfort_a_long, t, delta_t, dyn_obs, best_traj_ch, best_traj_cost_ch, best_traj_col_ch, best_traj_vp_ch, best_traj_vp_cost_ch, best_vp_accel_ch, best_vp_finalvel_ch);
						
							if (best_traj_col_ch==-1)
							{
								new_controller_update = 1; //% New path to the controller
								mandatory_point.point = final_point;
								mandatory_point.reason = 1; // 1--> maneuver of changing lane
								mandatory_point.traj = best_traj_ch;
								mandatory_point.GP_path_station = target_i_GP + best_traj_ch.traj[0].size();

								best_vp = best_traj_vp_ch;
								vp_accel = best_vp_accel_ch;
								vp_finalvel = best_vp_finalvel_ch;
								best_traj = best_traj_ch;
								
							}
						
						}
						
					}
				}
		   }
		   else if (possible_lanes[2]==1) // % Right lane change possible 
		   {
			   
		   }
	   }
		
	   // *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
       // *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
		
			
	   int num_traj=6; // number of trajectories generated in each manoeuvre 
	
	   if ( (dyn_obs_avoidance > 0) && (mandatory_point.reason == 0) && (best_traj_col == -1) ) //% Only if we are not in other manoeuvres and our current lane is clear
	   {
			// First of all, we search for feasibility in the LEFT lane up to our lookahead dist (mirem estat d'accesibilitat del carril esquerre)
			C_LaneChange new_lane_change ( LP_paths[1], LP_paths[0], resolution, sampling_distance, dists, offsets, initial_point, target_i_GP, static_obs, car_3circle_r, min_turn_r);
		
			int other_lane_blocked=new_lane_change.lane_change_is_blocked(); // % Mandatory checking // mirem si esta bloquejat (1--> estaq bloquejat)
		
			if (other_lane_blocked==0) //% Only continue if we can
			{
				float max_pos_empty[2]={-1,-1};
				vector<C_Traj> trajs_ch;
				vector<float> c_lengths_ch;
				vector<float> c_maxks_ch;
				vector<float> c_maxkdots_ch;
				vector<float> c_static_obs_ch;
				vector<float> collisions_ch;
				int lane_change_possible;
				C_SamplePoint final_point;
			
				new_lane_change.gen_lane_change_trajs (num_traj, best_traj, max_pos_empty, sampling_distance, trajs_ch, c_lengths_ch, c_maxks_ch, c_maxkdots_ch, c_static_obs_ch, collisions_ch, lane_change_possible, final_point);
			
				if (lane_change_possible == 1)
				{
					C_Traj best_traj_ch;
					float best_traj_cost_ch;
					float best_traj_col_ch;
					vector<vector<float> > best_traj_vp_ch;
					float best_traj_vp_cost_ch;
					float best_vp_accel_ch; 
					float best_vp_finalvel_ch;
					new_lane_change.best_lane_change_traj ( trajs_ch, c_lengths_ch, c_maxks_ch, c_maxkdots_ch, c_static_obs_ch, collisions_ch, comfort_a_lat, max_vel_restriction, vel, max_road_vel, accel, accel_last, -max_braking_accel, comfort_a_long, t, delta_t, dyn_obs, best_traj_ch, best_traj_cost_ch, best_traj_col_ch, best_traj_vp_ch, best_traj_vp_cost_ch, best_vp_accel_ch, best_vp_finalvel_ch);
					
				
					float max_cost_allowed =1000;
					if ( (best_traj_col_ch == -1) && (best_traj_vp_cost_ch < max_cost_allowed) ) // % No collision, neither static nor dynamic
					{
						// It seems we can avoid the dynamical obstacle
						// % Then, we generate the avoidances:
						
						vector<C_Traj> trajs_av;
						vector<float> c_lengths_av;
						vector<float> c_maxks_av;
						vector<float> c_maxkdots_av;
						vector<float> c_static_obs_av;
						vector<float> collisions_av;
						int lane_change_possible;
						new_lane_change.gen_dyn_avoidance ( num_traj, best_traj, dists, offsets, static_obs, trajs_av, c_lengths_av, c_maxks_av, c_maxkdots_av, c_static_obs_av, collisions_av, lane_change_possible);  

						if (lane_change_possible==1)
						{
							C_Traj best_traj_ch;
							float best_traj_cost_ch;
							float best_traj_col_ch;
							vector<vector<float> > best_traj_vp_ch;
							float best_traj_vp_cost_ch;
							float best_vp_accel_ch; 
							float best_vp_finalvel_ch;
							new_lane_change.best_lane_change_traj ( trajs_av, c_lengths_av, c_maxks_av, c_maxkdots_av, c_static_obs_av, collisions_av, comfort_a_lat, max_vel_restriction, vel, max_road_vel, accel, accel_last, -max_braking_accel, comfort_a_long, t, delta_t, dyn_obs, best_traj_ch, best_traj_cost_ch, best_traj_col_ch, best_traj_vp_ch, best_traj_vp_cost_ch, best_vp_accel_ch, best_vp_finalvel_ch);
							
							// Avoiding the dynamical obstacle
							
							if (best_traj_col_ch == 0)
							{
								float final_point_coords[2];
								final_point_coords[0]=best_traj.end_point_sampling_coords[0]; //[station]
								final_point_coords[1]=best_traj.end_point_sampling_coords[1]; //[offset]
								
								final_point= traj_points[final_point_coords[0]][final_point_coords[1]]; 
								
								new_controller_update = 1; //% New path to the controller
								
								mandatory_point.point = final_point;
								mandatory_point.reason = 5; //% Dynamic obstacle avoidance
								mandatory_point.traj = best_traj_ch;
								mandatory_point.GP_path_station = target_i_GP + best_traj_ch.traj[0].size();
								
								best_vp = best_traj_vp_ch;
								vp_accel = best_vp_accel_ch;
								vp_finalvel = best_vp_finalvel_ch;
								best_traj = best_traj_ch;
								
								dyn_obs_avoidance = 0; //% Remove the flag
							}
						}
					}	
				}
			}
		}
	
		// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
		// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
	
		// Return to the desired lane
			
		if ( ( desired_lane != current_lane) && (desired_lane>=0) && (desired_lane<Highway.num_lanes) )
		{
			 cout << 10 << endl ;
			if ( (desired_lane>current_lane) && (possible_lanes[0]==1) ) // % Left lane change possible
			{
				lane_change = 1; //% L --> we change to left lane
			}
			else if ( (desired_lane<current_lane) && (possible_lanes[2]==1) ) // % Right lane change possible
			{
				lane_change = -1; //% R --> we change to right lane
			}
		}
		
		if ( (lane_change != 0) && (mandatory_point.reason==0) ) // % We need to do a lane change and we are not doing one
		{
			C_LaneChange new_lane_change ( LP_paths[1], LP_paths[1-lane_change], resolution, sampling_distance, dists, offsets, initial_point, target_i_GP, static_obs, car_3circle_r, min_turn_r); 
			int other_lane_blocked=new_lane_change.lane_change_is_blocked(); // % Mandatory checking 
		
			if ( (other_lane_blocked==0) ) // % Only continue if we can
			{
				num_traj=8;
				
				float max_pos_empty[2]={-1,-1};
				vector<C_Traj> trajs_ch;
				vector<float> c_lengths_ch;
				vector<float> c_maxks_ch;
				vector<float> c_maxkdots_ch;
				vector<float> c_static_obs_ch;
				vector<float> collisions_ch;
				int lane_change_possible;
				C_SamplePoint final_point;
				new_lane_change.gen_lane_change_trajs (num_traj, best_traj, max_pos_empty, sampling_distance, trajs_ch, c_lengths_ch, c_maxks_ch, c_maxkdots_ch, c_static_obs_ch, collisions_ch, lane_change_possible, final_point);
				
				if (lane_change_possible==1)
				{
					C_Traj best_traj_ch;
					float best_traj_cost_ch;
					float best_traj_col_ch;
					vector<vector<float> > best_traj_vp_ch;
					float best_traj_vp_cost_ch;
					float best_vp_accel_ch; 
					float best_vp_finalvel_ch;
					new_lane_change.best_lane_change_traj ( trajs_ch, c_lengths_ch, c_maxks_ch, c_maxkdots_ch, c_static_obs_ch, collisions_ch, comfort_a_lat, max_vel_restriction, vel, max_road_vel, accel, accel_last, -max_braking_accel, comfort_a_long, t, delta_t, dyn_obs, best_traj_ch, best_traj_cost_ch, best_traj_col_ch, best_traj_vp_ch, best_traj_vp_cost_ch, best_vp_accel_ch, best_vp_finalvel_ch);
				
					if (best_traj_col_ch==-1)
					{
						current_lane = current_lane + lane_change;
						lane_change = 0;
					
						new_controller_update = 1; //% New path to the controller
						mandatory_point.point = final_point;
						mandatory_point.reason = 1; // 1--> maniobra de changing lane
						mandatory_point.traj = best_traj_ch;
						mandatory_point.GP_path_station = target_i_GP + best_traj_ch.traj[0].size();

						best_vp = best_traj_vp_ch;
						vp_accel = best_vp_accel_ch;
						vp_finalvel = best_vp_finalvel_ch;
						best_traj = best_traj_ch;
					}
				}
			}
		}
	
		//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

		// Velocity profile output---------------------------------------------------------------------------------------------------------

		float vel_index;
		vector<float> Punts;
		vector<float> Vels;
		vector<float> Accels;
	

		if ( (abs(vp_finalvel-last_vp_finalvel)>1) || ( (vp_finalvel == 0) && (vp_accel < last_vp_accel) ) )
		{
			vel_index = 0;
			Punts = best_vp[0];
			Vels = best_vp[1];
			Accels = best_vp[2];
			temps = best_vp[3];
				
			last_vp_finalvel = vp_finalvel;
			last_vp_accel = vp_accel;
			last_time = t;
		}
		
		// Trajectory (path) output---------------------------------------------------------------------------------------------------------------
		
		vector <vector<float> > LP_path_out; // vector with coordenates X, Y and k's
		
		if ( (new_controller_update == 1) || (target_j >= (last_traj[0].size())-((1.5*car_l/resolution)-1)) ) // % New obs update or arriving to the end
		{
			LP_path_out = best_traj.traj;
			last_traj = LP_path_out;
		}

		// Write output on files
		
		const char* path_output= "output_path.txt";
		write_path( path_output, LP_path_out); 	
			
		const char* vp_output= "output_vp.txt";
		write_vp( vp_output, Punts, Vels, Accels, temps); 
		
	/*
		vector<float> Punts1;
		vector<float> Vels1;
		vector<float> Accels1;
		vector<float> temps1;
			
		for (int i=0; i<100; i++)
		{
			Punts1.push_back(Punts[i]);
			Vels1.push_back(Vels[i]);
			Accels1.push_back(Accels[i]);
			temps1.push_back(temps[i]);
		}
		
		
			
		// Static obstacles activation and deactivation---------------------------------------------------------------------------------------------------------------
		 (plotejar)
		
		*/
		
		// prueba
		 /*
		for (int i=0; i<last_traj[1].size(); i++)
		{
			cout << last_traj[1][i] << endl ;
		}
		 */
		
		// Dynamic obstacles plot and activation/deactivation---------------------------------------------------------------------------------------------------------------

		for (int i=0; i<num_dyn_obs; i++)
		{
			dyn_obs[i].update_visibility(t);
			
		}

		// Highway behind---------------------------------------------------------------------------------------------------------------

		if ( (fmod(t,delta_t*50)==0) ) //¿??¿?50?¿?
		{
			for (int i=0; i<Highway.num_lanes; i++)
			{
				float target_i_aux;
				float dist3;
				
				Search_closest_point( pos_X, pos_Y, Highway.center_paths[i].traj[0], Highway.center_paths[i].traj[1], last_target_i_highway[i], target_i_aux, dist3); // millora de l'anterior
				last_target_i_highway[i] = target_i_aux;
			}
		}
		
		// Variable updating
		
		index_simu = index_simu + 1;
		last_target_i_GP = target_i_GP;
		//?¿?¿?pause(0.03) % MAIN PAUSE OF THE LOOP ********************************
		
   				
	
	} //ACTIVAR 'FOR' QUAN ACAVEM 

	
		
	
	//*********//
	
	
	return 0;

}
