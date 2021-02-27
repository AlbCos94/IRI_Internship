#include "C_Track.h" 

//constructor --> inicialitza l'objecte (nomes es fa un cop) --> executat automaticament quan un nou objecte es referenciat a ser d'aquesta classe --> permet inicialitzar variables membre d'aquesta classe, en definir noves variables d'aquesta classe haurem d'especificar els valors del constructor
C_Track::C_Track (float lane_width1, vector <C_Traj > center_paths1)
{
	lane_width=lane_width1;
	center_paths=center_paths1;
	num_lanes=(int) center_paths.size(); // calcula el numero de vectors que te la variables center_paths, la qual es un vector d'elements de la classe C_Traj (conte cadascun dels carrils)

	// calcul del numero de punts pel que esta formada cada trajectoria
	for (int i=0; i< (float) num_lanes; i++) // primer fem el seu valor absolut de tots els seus valors
	{ 
		num_points.push_back(center_paths[i].traj[0].size()); // calculem el numero de punts, amb les coordenades 'X''s que te (igual donaria per 'Y' que per curvatures)
	}
	
}
	



