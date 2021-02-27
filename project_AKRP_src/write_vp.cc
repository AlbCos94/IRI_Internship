#include "write_vp.h" 

using namespace std;

void write_vp(const char* file_name, vector<float> Punts, vector<float> Vels, vector<float> Accels, vector<float> temps) 
{
	
	ofstream vp; // creamos un objeto asociado a la clase de 'ofstream' para escribir datos en ficheros
	vp.open (file_name); // abrimos el fichero de nombre '  ' en modo por defecto de la clase ofstream --> ios::out	(Open for output operations)
	
	for (int i=0; i< (float) Punts.size(); i=i+1) //first line
	{
		vp << Punts[i];
		vp << " ";
	}
	vp << ".\n";
	
	for (int i=0; i< (float) Vels.size(); i=i+1) //second line
	{
		vp << Vels[i];
		vp << " ";
	}
	vp << ".\n";
	
	for (int i=0; i< (float) Accels.size(); i=i+1) // Third line
	{
		vp << Accels[i];
		vp << " ";
	}
	vp << ".\n";
	
	for (int i=0; i< (float) temps.size(); i=i+1) // Fourth line
	{
		vp << temps[i];
		vp << " ";
	}
	vp << ".\n";
	
	
	vp.close(); 
}
