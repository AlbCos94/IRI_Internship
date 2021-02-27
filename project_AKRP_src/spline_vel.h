#ifndef __SPLINE_VEL_H_INCLUDED__
#define __SPLINE_VEL_H_INCLUDED__

#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>    // std::max

//propies
#include "funcions_mat.h" 

using namespace std;

//Conjunto de funciones para crear los perfiles de velocidad

//funcio de generacio de perfils de velocitats sense acceleració inicial 
void spline_3rdo_vel(float a_acc,float a_fre,float v_0, float v_f, float x_0, float t_0, float delta_t, float min_end_pos, std::vector<float>& Punts, std::vector<float>& Vels, std::vector<float>& Accels, std::vector<float>& temps);

//funcio de generacio de perfils de velocitats amb acceleració inicial 
void spline_3rdo_vel_a0(float a_acc, float a_fre, float var_acc, float a_0, float v_0, float v_f, float x_0, float t_0, float delta_t, float min_end_pos, std::vector<float>& Punts, std::vector<float>& Vels, std::vector<float>& Accels, std::vector<float>& temps);
					// inputs
						//a_acc --> acceleracio maxima a la que es pot arrivar en aquell perfil
						//a_fre --> desacceleracio maxima a la que es pot arrivar en aquell perfil
						//var_acc  --> terme independent del polinomi de la derivada de l'acceleracio
						//a_0 --> acceleracio inicial
						//v_0 --> velocitat inicial
						//v_f --> velocitat final
						//x_0 --> distancia inicial
						//t_0 --> instant inicial de temps
						//delta_t --> temps de mostreig
						// min_end_pos --> posicio minima a arrivar
					// outputs
						// Punts --> vector amb les diferent posicion a cada instant
						// Vels --> vector amb les diferent velocitats a cada instant
						// Accels --> vector amb les diferents acceleracions a cada instant
						// temps --> vector amb els instants de temps

//funcio de generacio de perfils de velocitats (amb la coordenada de l'estacio final fixada) emprada pel filtratge de candidats	
void spline_3rdo_velX(float v_0, float v_f, float x_0, float x_f, float t0, float delta_t, std::vector<float>& Punts, std::vector<float>& Vels, std::vector<float>& Accels, std::vector<float>& temps);

#endif // !__SPLINE_VEL_H_INCLUDED__

