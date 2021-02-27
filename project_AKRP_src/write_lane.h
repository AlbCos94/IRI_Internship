#ifndef __WRITE_LANE_H_INCLUDED__
#define __WRITE_LANE_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>

#include <sstream>
#include <string> 
#include <stack>
#include <fstream> 

//Propies
#include "G2_Spline.h" 


using namespace std;


// crea un fitxer .txt que representa un carril amb el seguent format

// x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 .... (coordenades en X dels punts conformen la linia central del carril)
// y1 y2 y3 y4 y5 y6 y7 y8 y9 y10 y11 y12 y13 .... (coordenades en Y dels punts conformen la linia central del carril)
// c1 c2 c3 c4 c5 c6 c7 c8 c9 c10 c11 c12 c13 .... (curvatures del carril en aquests punts)
// path_dist (distancia longitudinal del carril

// emprant la funcio G2_Spline
 
void write_lane(const char* file_name, float x_A, float y_A, float K_A, float theta_A, float x_B, float y_B, float K_B, float theta_B, int N, float resolution); 

// file_name --> nom del fitxer que es creara

// dades del punt inicial del centre del carril
	//x_A, y_A, K_A, theta_A
	
// dades del punt final del centre del carril
	//x_B, y_B, K_B, theta_B
	
//numero de iteracioons dels parametres de la Spline
	// N
	


#endif // !__READ_LANE_H_INCLUDED__
