#ifndef __CHECK_COL_SAMPLE_POINT_H_INCLUDED__
#define __CHECK_COL_SAMPLE_POINT_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>

//propies
#include "C_SamplePoint.h" 
#include "C_Obstacle.h" 

using namespace std;


// determina si en una posicio donada del vehicle aquest colisionara amb un cert obstacle 
int check_col_sample_point (C_SamplePoint sample_point, float r, C_Obstacle obs); 
						// input
							// samle_point --> objecte de C_SamplePoint que determina la posicio del cotxe
							// r --> radi que representa la circunferencia amb que es determina el cotxe
							// obs --> objecte de C_Obstacle que determina el obstacle en questio
						// output --> retorna un 1 si hi ha colisio, retorna un 0 en cas contrari

#endif // !__CHECK_COL_SAMPLE_POINT_H_INCLUDED__
