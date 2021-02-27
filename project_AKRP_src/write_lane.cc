#include "write_lane.h" 

using namespace std;

void write_lane(const char* file_name, float x_A, float y_A, float K_A, float theta_A, float x_B, float y_B, float K_B, float theta_B, int N, float resolution) 
{
	
	
	//outputs de la funcio G2_Spline
	vector<float> Px;
	vector<float> Py;
	float path_dist;
	vector<float> curvature;
	
	G2_Spline(x_A, y_A, K_A, theta_A, x_B, y_B, K_B, theta_B, N, resolution, Px, Py, path_dist, curvature);
	
	
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
	
	carril << path_dist; // tambe escrivim en el fitxer la longitud del carril (encara que despres no la fem servir ?¿?¿)
	
	carril.close(); 
}
