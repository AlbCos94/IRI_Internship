#include "C_SamplePoint.h" 


using namespace std; 

//constructor 
C_SamplePoint::C_SamplePoint (float x1, float y1, float k1, float heading1, float s1)
{
	x=x1;
	y=y1;
	k=k1;
	head=heading1;
	s=s1; // per defecte es fara que s1=-1
}

//default constructor 
C_SamplePoint::C_SamplePoint ()
{
	x=0;
	y=0;
	k=0;
	head=0;
	s=-1; // per defecte es fara que s1=-1
}
