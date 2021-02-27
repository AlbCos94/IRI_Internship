#ifndef __CAR_CIRCLES_H_INCLUDED__
#define __CAR_CIRCLES_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>

using namespace std;


// dona el radi i els centres dels cercles que representen el cotxe
void car_circles (float car_l, float  car_w, float  car_r_ovh, float  pos_X, float  pos_Y, float  heading, int n, float& r, vector <vector<float> >& centers); 

// r --> radi dels cercles que defineixen el cotxe
// centers --> conjunt de vectors que representen la situcio dels centres, el primer component sera la coordenada X i la segona component sera la coordenada Y

#endif // !__CAR_CIRCLES_H_INCLUDED__
