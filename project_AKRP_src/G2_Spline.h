#ifndef __G2_SPLINE_H_INCLUDED__
#define __G2_SPLINE_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>


#include "funcions_mat.h" 
using namespace std;

// dona les cordenades en X i Y, la distancia i la curvatura que defineix la spline que uneix dos punts 
void G2_Spline(float x_A, float y_A, float K_A, float theta_A, float x_B, float y_B, float K_B, float theta_B, int N, float resolution, std::vector<float>& Px, std::vector<float>& Py, float& path_dist, std::vector<float>& curvature);
				// inputs
					// x_A , y_A --> coordenades punt inicial
					// K_A --> curvatura punt inicial
					// theta_A --> orientacio inicial [en radians]
					// x_B , y_B --> coordenades punt final
					// K_B --> curvatura punt final
					// theta_B --> orientacio final en radians]
					// N --> numero de iteracions per trobar els coeficients de la 'spline'
					// resolution --> metres entre punts de la discretitzacio
				// outputs
					// Px, Py vectors amb les coordenades X i Y de la trajectoria generada
					// path_dist --> distancia longitudinal del cami
					// curvature --> valor de la curvatura a cada punt

#endif // !__G2_SPLINE_H_INCLUDED__
