#ifndef __C_STATICOBS_H_INCLUDED__
#define __C_STATICOBS_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>  

//propies
#include "C_Obstacle.h" 
#include "Search_closest_point.h" 

using namespace std; 

//Creacio classe per englobar el conjunt d'obstacles de la carretera --> contindra elements de la clase C_Obstacle
class C_StaticObs 
{
	public: 
		// definim els membre de la clase (tots seran d'acces public)
		vector <C_Obstacle > list; // vector d'elements de la clase C_Obstacle els quals representen els obstacles
		int num_static_obs; // numero de obstacles estatics que hi han a la carretera
		
		vector <float > visibility; // vector amb 1's o 0's corresponents als objectes (igual numero que d'objectes)
									// 1 --> l'obstacle esta dins del rang de visibilitat del sensor
									// 0 --> l'obstacle NO esta dins del rang de visibilitat del sensor
		vector <float > station; // vector amb les coordenades longitudinals de les 'stations' (estacions) del carril
				//stations REPRSENTADES AMB EL NUMERO DE PUNT del trajecte, ULL! --> comença amb el 0 a diferencia de amb el matlab que comença per l'1
					
		vector <vector<float> > GP_path; // current Gloabal Planner path --> carril que es vol seguir per defecte
											// 1er vector --> coordenades X
											// 2on vector --> coordenades Y
											// 3er vector --> curvatures
					
											
		//constructor 
		C_StaticObs(vector <vector<float> > GP_path1); //s'inicialitza en introduir el carril on volem situar els obstacles on:
		
																												// 1er vector --> coordenades X
																												// 2on vector --> coordenades Y
																												// 3er vector --> curvatures
		//constructor per defecte
		C_StaticObs();
		
		
		
		//funcions membre
		void add_obs(C_Obstacle obs1); // funcio per a afegir nous obstacles a la carretera
		
		int update_visibility( float lookahead_dist, float our_pos_X, float our_pos_Y, float our_station); // actualitza el vector de 'visibility' que representa quins obstacles son visibles pel cotxe i quins no  
																										   // retorna 1 si ha modificat el vector de visibilitat
		void set_new_GP_path( vector <vector<float> > new_GP_path); // asignem un carril diferent en el cual volem situar els obstacles
		
		void get_closest_obs(vector <C_Obstacle> &obstacle, float &obs_station, int &no_obstacle); // retorna un vector de objectes de la clase C_Obstacle visibles que es troben mes aprop al cotxe (normalment nomes trobara 1, a no ser que hi hagin dos associats a la mateixa 'station') , amb la 'station' en la que es troba/en ('numero de punt del carril') i si no hi ha cap ( o encara no els visualitza) 'no_obstacles' valdra 0, i si detecta algun valdra 1
		
		void get_closest_obs_from_pos(float target_i, vector <C_Obstacle> &obstacle, float &obs_station, int &no_obstacle); // semblant a la funcio anterior. Retorna el obstacle/s visible/s mes proper d'una certa 'station' (numero de punt del trajecte) que li quedi per davant (la 'station' asociada al objecte ha de ser superior a la 'station' indicada 'target_i')

		void get_all_closest_obs(vector <C_Obstacle> &obstacle, vector <float> &obs_station, int &no_obstacle); //  retorna un vector de tots els obstacles visibles i ordenats de mes a prot a mes lluny / tambe retorna el vector associat a les estacions d'aquests objectes (tambe ordenat) i una variable per indicar quan no hi ha cap objecte visible

		void get_all_closest_obs_interval(float target_i_ini, float target_i_fin, vector <C_Obstacle> &obstacle, vector <float> &obs_station, int &no_obstacle); // retorna un vector de tots els obstacles visibles i ordenats de mes aprop a més lluny i dins un interval de 'stations' indicat pel valors de 'station' major i el valor de 'station' menor tambe retorna el vector associat a les estacions d'aquests objectes (tambe ordenat) i una variable per indicar quan no hi ha cap objecte visible 

		int get_num_obs(void); // dona el numero d'obstacles que tenim en el carril
		
		C_Obstacle get_obs(int id, int &no_obstacle); // retorna el obstacle amb el 'id' indicat , si no existeix 'no_obstacle' valdra 0
};

#endif // !__C_STATICOBS_H_INCLUDED__
