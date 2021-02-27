#ifndef __SAMPLING_DIST_H_INCLUDED__
#define __SAMPLING_DIST_H_INCLUDED__

#include <math.h>
#include <iostream>

using namespace std;

// calculo la distancia que necesita de frenado, segun la velocidad que lleva el coche 
// determina la distancia de muestreo en que se discretiza dos punts del circuito, 
// depende de la velocidad que lleve el coche y su capacidad de frenado
float sampling_dist (float vel, float  max_braking_accel, float security_distance); 

// input elements
		// vel --> velocidad a la que va el coche
		// max_breaking_accel --> maxima aceleracion de frenado que tiene el coche
		// security_distance --> ditancia en metres que afegim de mes per a asegurar la frenada

// output elements
		// brk_dist --> distancia que necesita de frenada, la cual tambien sera la distancia de muestreo de la carretera
			// contra mes a poc a poc anem, agafarem una distancia de mostreig menor 
// TFM pg 42 explicat

#endif // !__SAMPLING_DIST_H_INCLUDED__
