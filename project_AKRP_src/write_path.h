#ifndef __WRITE_PATH_H_INCLUDED__
#define __WRITE_PATH_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>

#include <sstream>
#include <string> 
#include <stack>
#include <fstream> 

//Propies



using namespace std;


// crea un fitxer .txt que representa un perfil de velocitat amb el seguent format

// x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 .... (Xs coordinates of the path)
// y1 y2 y3 y4 y5 y6 y7 y8 y9 y10 y11 y12 y13 .... (Ys coordinates of the path)
// c1 c2 c3 c4 c5 c6 c7 c8 c9 c10 c11 c12 c13 .... (curvatures of the lane)


void write_path(const char* file_name, vector <vector<float> > path); 

	


#endif // !__WRITE_PATH_H_INCLUDED__
