#ifndef __STATIC_OBS_FUNCTION_H_INCLUDED__
#define __STATIC_OBS_FUNCTION_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>

using namespace std;

// retorna el cost referenciat a la distancia del vehicle als obstacles
// si la distancia del vehicle al obstacle es menor a un cert llindar, el cost es veurà greument penalitzat
// dist=distancia de la periferia del cercle que representa el vehicle fins a la periferia del cercle que representa el obstacle
// dist= dist(centre cercle vehicle - centre cercle obstacle)-radi_vehicle-radi_obstacle
// funcio explicada a TFM pg62
float static_obs_function (float dist);

//retorna un float que es el cost

#endif // !__STATIC_OBS_FUNCTION_H_INCLUDED__
