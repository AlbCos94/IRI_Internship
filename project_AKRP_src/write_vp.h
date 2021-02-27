#ifndef __WRITE_VP_H_INCLUDED__
#define __WRITE_VP_H_INCLUDED__

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

// x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 .... (coordenades longitudinals associats al desplaçament produit pel perfil de velocitat)
// v1 v2 v3 v4 v5 v6 v7 v8 v9 v10 v11 v12 v13 .... (velocitats per a cada instant de temps (m/s))
// a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11 a12 a13 .... (acceleracions per a cada instant de temps)
// t1 t2 t3 t4 t5 t6 t7 t8 t9 t10 t11 t12 t13 .... (instants de temps)


void write_vp(const char* file_name, vector<float> Punts, vector<float> Vels, vector<float> Accels, vector<float> temps); 

	


#endif // !__WRITE_VP_H_INCLUDED__
