#ifndef __MAX_ADMISSIBLE_VELS_H_INCLUDED__
#define __MAX_ADMISSIBLE_VELS_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>  

//propies
#include "funcions_mat.h" 


using namespace std;

//
void max_admissible_vels (vector<float> LP_path_k, float resolution, float comfort_a_lat, float max_road_vel, float (&max_vel_restriction)[2]);
							
						// input
							// LP_path_k --> vector de curvatures del carril actual que ens trobem 
							// resolution --> resolucio
							// comfort_a_lat --> acceletacio lateral de comfort
							// max_road_vel --> máxima velocitat permesa en la carretera (m/s)
						//output
							// max_vel_restriction[2] --> array de dos elements
									// 1er element --> distancia en metres longitudinals a la carretera a partir de la qual es fara la restriccio de velocitat 
													 //fins a aquests metres s'establira com a maxima velocitat permessa, la propia de la carretera
									// 2on element --> velocitat maxima admisible



#endif // !__MAX_ADMISSIBLE_VELS_H_INCLUDED__
