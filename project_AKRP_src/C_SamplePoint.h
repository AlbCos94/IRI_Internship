#ifndef __C_SAMPLEPOINT_H_INCLUDED__
#define __C_SAMPLEPOINT_H_INCLUDED__

#include <math.h>
#include <iostream>

using namespace std; 

//Creacio classe per caracteritzat el punt de situacio del cotxe (estat espaial)
class C_SamplePoint 
{
	public: 
		// definim els membre de la clase (tots seran d'acces public)
		float x; // coordenada en X
		float y; // coordenada en Y
		float k; // curvatura 1/m
		float head; // orientacio cotxe (degrees)		
		float s; // 'station' numero del punt del carril en que s'ha discretitzat aquest
				
		//constructor 
		C_SamplePoint (float x1, float y1, float k1, float heading1, float s1); 
		//default cpnstructor
		C_SamplePoint(); // inicialitza tot a 0, menys la 's' que la posa a -1
		
};

#endif // !__C_SAMPLEPOINT_H_INCLUDED__
