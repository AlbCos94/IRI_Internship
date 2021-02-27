#include "C_Traj.h" 


//constructor --> inicialitza l'objecte (nomes es fa un cop) --> executat automaticament quan un nou objecte es referenciat a ser d'aquesta classe --> permet inicialitzar variables membre d'aquesta classe, en definir noves variables d'aquesta classe haurem d'especificar els valors del constructor
C_Traj::C_Traj (float ini_point_sampling_coords1[2], float end_point_sampling_coords1[2], vector <vector<float> > traj1, float dist1, float res1) 
{
	ini_point_sampling_coords[0]=ini_point_sampling_coords1[0];
	ini_point_sampling_coords[1]=ini_point_sampling_coords1[1];

	end_point_sampling_coords[0]=end_point_sampling_coords1[0];
	end_point_sampling_coords[1]=end_point_sampling_coords1[1];

	traj=traj1;

	resolution=res1;

	len=dist1;
	
	
	// calcul de maxk
	vector<float> curv=traj[2]; // creem vector de curvatures per despres trobar el seu maxim en valor absolut

	for (int i=0; i< (float) curv.size(); i++) // primer fem el seu valor absolut de tots els seus valors
	{ 
		curv[i]=abs(curv[i]);
	}

	
	maxk = *max_element(curv.begin(), curv.end()); //treien el element mes elevat del vector de curvatures de la matriu 'traj1' --> treiem curvatura maxima --> maxk
	
	
	//calcul de maxkdot --> increment de curvatura maxim
	vector<float> diff;
	for (int i=0; i< (int) (traj[2].size()-1); i++) // primer fem el seu valor absolut de tots els seus valors
	{
		diff.push_back( (traj[2][i+1]-traj[2][i]) ) ; // vector de diferencies entre curvatures adjacents
	}
	
	for (int i=0; i< (float) diff.size(); i++) // fem el seu valor absolut de tots els seus valors
	{ 
		diff[i]=abs(diff[i]);
	}
	
	 maxkdot=max_value(diff); //extreiem el valor maxim

}

//default constructor 
C_Traj::C_Traj ()
{
	ini_point_sampling_coords[0]=0;
	ini_point_sampling_coords[1]=0;

	end_point_sampling_coords[0]=0;
	end_point_sampling_coords[1]=0;

	vector<float> traj1_aux;
	traj1_aux.push_back(0);
	traj1_aux.push_back(0);
	traj1_aux.push_back(0);
	
	vector <vector<float> > traj2_aux;
	
	traj2_aux.push_back(traj1_aux);
	
	traj=traj2_aux;

	resolution=0;

	len=0;

	maxk = 0; 
	
	
}


	
	
//escrivim les funcions que estan definides com a membres de funcio de la classe C_Traj 
void C_Traj::compute_cost_static_obs(vector <C_Obstacle> obstacle, float r_col, float &cost, float &col_index)
{
	col_index=-1; // amb aquest valor s'indica que no hi ha estacio on es produeixi colisio
	int num_obs=obstacle.size(); //numero d'obstacles que hi han
	cost=0;
	float threshold_value=1000; // valor llindar del cost, a partir del qual considerarem que es produeix colisio
	
	for (int i=0; i< num_obs; i++) // per a cada obstacle que tenim en el cami, calculem el cost estatic que aquest suposa per a la trajectoria
	{ 
		
		float target_i; // --> es el ''numero de punt'' (station) mes proper del punt estudiat (pos_X,pos_Y) al conjunt de punts (path_X, path_Y)
		float dist; // --> es la distancia del punt estudiat al punt mes proper del conjunt de punts 
		//Busquem la 'station' de la trajectoria que esta mes aprop de cada obstacle, i la distancia fins a entre ells
		Search_closest_point_full(obstacle[i].pos[0], obstacle[i].pos[1], traj[0], traj[1], 0, target_i, dist);
		
		float c; //cost estatic asociat del cotxe respecte aquell obstacke
		
		c= static_obs_function(dist-r_col-obstacle[i].r); // la distancia que separa obstacle i vehicle es la que hi ha entre periferies dels contorns dels cercles que representen cadascun
		
		cost=cost+c; // acumulem el cost que representen el conjunt dels obstacles
		
		if ( (c>=threshold_value) && ( (col_index == -1) || ((col_index > -1) && (target_i < col_index)) ) ) // si el cost associat a aquell objcte es molt elevat i no s'haviat trobat un altre obstacle amb cost asociat tant elevat mes proper
		{
			col_index= target_i; // indiquem la 'station' amb un gran cost asociat (risc de colisio) 
		}
	}
	cost_s=cost; // asociem el cost total amb el membre de la classe de cost estatic de la trajectoria
}

float C_Traj::get_static_cost(void)
{
	return cost_s;
}


