#ifndef __SEARCH_CLOSEST_POINT_H_INCLUDED__
#define __SEARCH_CLOSEST_POINT_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>

using namespace std;

//calcula la minima distancia d'un punt (x,y) a un altre conjunt de punts
//retorna a traves de la modificacio de dues variables el ''numero de punt'' (station) mes proper de la caretera i la distancia fins aquest
void Search_closest_point_full(float pos_X, float pos_Y, vector <float> path_X, vector <float> path_Y, float start_i, float& target_i, float& dist);

void Search_closest_point(float pos_X, float pos_Y, vector <float> path_X, vector <float> path_Y, float start_i, float& target_i, float& dist); // millora de l'anterior
//input
	//pos_X , pos_y --> coordenades del punt estudiat (sera les coordenades X i Y del cotxe)
	//path_X --> coordenades X de un conjunt de punts (vector de coordenades X)
	//path_Y --> coordenades Y de un conjunt de punts (vector de coordenades Y)
	//start_i --> es el ''numero de punt'' del conjunt de punts a partir del qual comencen a mirar aquesta distancia (ens permet calcular la minima distancia a a una part d'aquest conjunt de punts) 

//output (s'hauran de declarar abans de cridar la funcio)
	//target_i --> es el ''numero de punt'' (station) de la carretera, mes proper al punt estudiat (pos_X,pos_Y) (posicio del cotxe o obstacle)
	//dist --> es la distancia del punt estudiat al punt mes proper del conjunt de punts 


#endif // !__SEARCH_CLOSEST_POINT_H_INCLUDED__
