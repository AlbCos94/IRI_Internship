#ifndef __READ_LANE_H_INCLUDED__
#define __READ_LANE_H_INCLUDED__

#include <math.h>
#include <vector>
#include <iostream>

#include <sstream>
#include <string> 
#include <stack>
#include <fstream> 

using namespace std;


// llegeix un fitxer .txt amb el seguent format
// x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 .... (coordenades en X dels punts conformen la linia central del carril)
// y1 y2 y3 y4 y5 y6 y7 y8 y9 y10 y11 y12 y13 .... (coordenades en Y dels punts conformen la linia central del carril)
// c1 c2 c3 c4 c5 c6 c7 c8 c9 c10 c11 c12 c13 .... (curvatures del carril en aquests punts)

// proporciona un vector de 3 vectors, on un vector conte coordenades en X, un altre coordenades en Y i l'ultim les curvatures
void read_lane(const char* nom_fitxer, vector <vector<float> >& GP_path); 


// per a llegir el ultim valor del fitxer, que es unic i correspon a la longitud del carril
void read_path_dist(const char* nom_fitxer, float &path_dist); 


#endif // !__READ_LANE_H_INCLUDED__
