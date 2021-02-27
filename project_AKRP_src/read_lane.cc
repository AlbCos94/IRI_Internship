#include "read_lane.h" 

using namespace std;

void read_lane(const char* nom_fitxer, vector <vector<float> >& GP_path)
{
	ifstream fitxerCarretera;  // ifstream fitxerCarretera ("GP_path.txt");
	fitxerCarretera.open(nom_fitxer);
	
	float value; 
	int c=0;
	string line;
	
	while (getline(fitxerCarretera,line))
	{
		vector<float> temp; // vector auxiliar per anar construint el vector de vectors 
		
		if (c==0) //valors de X
		{
			istringstream iss(line); 
			while (iss >> value)
			{
				temp.push_back(value);
			}
			GP_path.push_back(temp);
		}
		else if (c==1) //valors de Y
		{
			istringstream iss(line); 
			while (iss >> value)
			{
				temp.push_back(value);
			}
			GP_path.push_back(temp);
		}	
		else if (c==2) //valors de Curvatura
		{
			istringstream iss(line); 
			while (iss >> value)
			{
				temp.push_back(value);
			}
			GP_path.push_back(temp);
		}
		else if (c==3) 
		{ 
			break; 
		} 
		c=c+1;
		temp.clear();
	}

}

void read_path_dist(const char* nom_fitxer, float &path_dist)
{
	ifstream fitxerCarretera;  // ifstream fitxerCarretera ("GP_path.txt");
	fitxerCarretera.open(nom_fitxer);
	
	float value; 
	int c=0;
	string line;
	
	while (getline(fitxerCarretera,line))
	{
		if (c==3) //valor de path_distance
		{
			istringstream iss(line); 
			
			iss>> value;
			path_dist=value;
			
			/*
			while (iss >> value)
			{
				path_dist=value;
			}
			*/
		}
		c=c+1;
	}
}
