#include "write_path.h" 

using namespace std;

void write_path(const char* file_name, vector <vector<float> > path) 
{
	
	vector<float> Px=path[0];
	vector<float> Py=path[1];
	vector<float> curvature=path[2];
	
		
	ofstream carril; // creamos un objeto asociado a la clase de 'ofstream' para escribir datos en ficheros
	carril.open (file_name); // abrimos el fichero de nombre '  ' en modo por defecto de la clase ofstream --> ios::out	(Open for output operations)
	
	for (int i=0; i< (float) Px.size(); i=i+1)
	{
		carril << Px[i];
		carril << " ";
	}
	carril << ".\n";
	
	for (int i=0; i< (float) Py.size(); i=i+1)
	{
		carril << Py[i];
		carril << " ";
	}
	carril << ".\n";
	
	for (int i=0; i< (float) curvature.size(); i=i+1)
	{
		carril << curvature[i];
		carril << " ";
	}
	carril << ".\n";
	
		
	carril.close(); 
}
