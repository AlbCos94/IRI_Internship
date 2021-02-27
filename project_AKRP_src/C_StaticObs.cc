#include "C_StaticObs.h" 


using namespace std; 

//constructor --> inicialitza l'objecte (nomes es fa un cop) --> executat automaticament quan un nou objecte es referenciat a ser d'aquesta classe --> permet inicialitzar variables membre d'aquesta classe, en definir noves variables d'aquesta classe haurem d'especificar els valors del constructor
C_StaticObs::C_StaticObs (vector <vector<float> > GP_path1)
{
	num_static_obs=0;
	GP_path=GP_path1;
}

//constructor per defecte --> vector de 0s
C_StaticObs::C_StaticObs ()
{
	num_static_obs=0;
	vector <vector<float> > GP_path1;
	vector <float > path_aux_X;
	vector <float > path_aux_Y;
	vector <float > path_aux_k;

	
	path_aux_X.push_back(0);
	path_aux_Y.push_back(0);
	path_aux_k.push_back(0);
	
	GP_path1.push_back(path_aux_X);
	GP_path1.push_back(path_aux_Y);
	GP_path1.push_back(path_aux_k);

	
	GP_path=GP_path1;
}

//escrivim les funcions que estan definides com a membres de funcio de la classe C_Traj 

void C_StaticObs::add_obs (C_Obstacle obs1) 
{
	num_static_obs=num_static_obs+1; // augmentem en 1 el nombre de obstacles que hi han
	list.push_back(obs1); // afegim l'obstacle a la llista de obstacles
	visibility.push_back(0); // quan creem el objecte, aquest es presuposa que encara no es visible, despres haurem de executar la funcio 'update_visibility' per veure si ja l'estem captant
	
	float target_i_GP; //'numero del punt' del carril mes proxim al obstacle introduit
	float dist; // distancia minima de l'obstacle als punts que conformen la carretera
	
	Search_closest_point_full(obs1.pos[0], obs1.pos[1], GP_path[0], GP_path[1], 0, target_i_GP, dist); // al començar el punt inicial es el 'numero de punt' 0
	//Search_closest_point_full(posX, posY, path_X, path_Y, start_i, target_i, dist);
	station.push_back(target_i_GP); //afegim una estacio ('numero de punt del carril') que correspondra al punt ('numero de punt') més aprop del carril a aquell obstacle
}

int C_StaticObs::update_visibility (float lookahead_dist, float our_pos_X, float our_pos_Y, float our_station) 
{
	int change=0;
	
	for (int i=0; i<(float)(num_static_obs); i=i+1)
	{
		float dist = sqrt(pow((list[i].pos[0] - our_pos_X),2) + pow((list[i].pos[1] - our_pos_Y),2)); //calculo la distancia de la posicio del cotxe fins el centre de cada obstacle que he registrat
		
		if ((visibility[i]==0) && (dist <= lookahead_dist) && (station[i] >= our_station)) // si es tenia registrat que el obstacle encara no era visible ; la distancia de captacio del cotxe (lookahead_dist) es mes gran o igual a la distancia que ens trobem del obstacle ; i encara no em arrivat al punt de la estacio que correspon a aquell obstacles (ens trobem per endarrere)), llavors indiquem que aquell obstacle ja es visible, posant la component corresponent del vector visibility a 1
		{
			visibility[i]=1;
			change=1;// indiquem que em canviat component/s del vector visibilitat
		}
		
		if ((visibility[i]==1) && (station[i] < our_station)) // si ja es trobava el objecte en el camp de capatacio del cotxe, pero la nostra posicio actual ('station') es superior a la que es troba el objecte, aixo voldra dir que ja l'hem sobrepassat, (ens queda per darrere) --> ja no el detectem --> posem el seu component de visibilitat a 0
		{
			visibility[i]=0;
			change=1;// indiquem que em canviat component/s del vector visibilitat
		}
	} 
	return change;
}

void C_StaticObs::set_new_GP_path( vector <vector<float> > new_GP_path)
{
	GP_path=new_GP_path; // assignem un carril diferent
	
	for (int i=0; i<(float)(num_static_obs); i=i+1)
	{
		float target_i_GP; //'numero del punt' del carril mes proxim al obstacle introduit
		float dist; // distancia minima de l'obstacle als punts que conformen la carretera
		Search_closest_point_full(list[i].pos[0], list[i].pos[1], GP_path[0], GP_path[1], 0, target_i_GP, dist); // al començar el punt inicial es el 'numero de punt' 0
		station[i]=target_i_GP;
	}
}

void C_StaticObs::get_closest_obs(vector <C_Obstacle> &obstacle, float &obs_station, int &no_obstacle)
{	
	no_obstacle=0; // continuara valent 0 si no en detecta cap obstacle
	obs_station=0;
	int ind=0; // indicador per saber si hi han obstacles que veiem
	
	for (int i=0; i<(float)(visibility.size()); i=i+1) // mirem primer si tenim algun obstacles en el camp de visualitzacio
	{
		if (visibility[i]==1)
		{
			ind=1; // detectem un obstacle que veiem
			break;
		}
	}
	
	if (ind>0) // hi han obstacles que visualitzem
	{
		// treiem un vector dels index de les estacions de forma que indiqui com es ordenen de forma creixent segons llunyania
		vector<int> index(station.size()); // vector que contindra els indexs de les estacions ordenats de forma creixent, segons llunyania d'aquestes
		size_t n(0);
		generate(std::begin(index), std::end(index), [&]{ return n++; });
		sort(std::begin(index),std::end(index),[&](int i1, int i2) { return station[i1] < station[i2]; } );
		
		for (int i=0; i<num_static_obs; i=i+1)
		{
			int j=index[i];
			if (visibility[j] == 1)
			{
				obstacle.push_back(list[j]); // afegim a la llista de obstacles propers el obstacle trobat
				obs_station=station[j]; // tambe asignem la seva estacio corresponent	
				no_obstacle=1; //retornem que hi ha algun obstacle que visualitzem
				
					if ((i+1)<num_static_obs) // per a chequejar el seguent obstacle visible
					{
						int j2=index[i+1];
						if (station[j]==station[j2]) // cas en que hi han dos obstacles en la mateixa 'station' de la carretera // mes de dos no podran haverhi ?¿??¿
						{
							obstacle.push_back(list[j2]);
						}
					}
				break;
			}
		}
	}
}

void C_StaticObs::get_closest_obs_from_pos(float target_i, vector <C_Obstacle> &obstacle, float &obs_station, int &no_obstacle)
{	
	no_obstacle=0; // continuara valent 0 si no en detecta cap obstacle
	obs_station=0;
	int ind=0; // indicador per saber si hi han obstacles que veiem
	
	for (int i=0; i<(float)(visibility.size()); i=i+1) // mirem primer si tenim algun obstacles en el camp de visualitzacio
	{
		if (visibility[i]==1)
		{
			ind=1; // detectem un obstacle que veiem
			break;
		}
	}
	
	if (ind>0) // hi han obstacles que visualitzem
	{
		// treiem un vector dels index de les estacions de forma que indiqui com es ordenen de forma creixent segons llunyania
		vector<int> index(station.size()); // vector que contindra els indexs de les estacions ordenats de forma creixent, segons llunyania d'aquestes
		size_t n(0);
		generate(begin(index), end(index), [&]{ return n++; });
		sort(begin(index),end(index),[&](int i1, int i2) { return station[i1] < station[i2]; } );
		
		for (int i=0; i<num_static_obs; i=i+1)
		{
			int j=index[i];
			if ((visibility[j] == 1) && (station[j]>target_i)) // mirarem que el objecte sigui visible y que estigui associat a una 'station' superior a la de la 'station' indicada
			{
				obstacle.push_back(list[j]); // afegim a la llista de obstacles propers el obstacle trobat
				obs_station=station[j]; // tambe asignem la seva estacio corresponent	
				no_obstacle=1; //retornem que hi ha algun obstacle que visualitzem
				
					if ((i+1)<num_static_obs) // per a chequejar el seguent obstacle visible
					{
						int j2=index[i+1];
						if (station[j]==station[j2]) // cas en que hi han dos obstacles en la mateixa 'station' de la carretera // mes de dos no podran haverhi ?¿??¿
						{
							obstacle.push_back(list[j2]);
						}
					}
				break;
			}
		}
	}
}

void C_StaticObs::get_all_closest_obs(vector <C_Obstacle> &obstacle, vector <float> &obs_station, int &no_obstacle)
{	
	no_obstacle=0; // continuara valent 0 si no en detecta cap obstacle
	int ind=0; // indicador per saber si hi han obstacles que veiem
	
	for (int i=0; i<(float)(visibility.size()); i=i+1) // mirem primer si tenim algun obstacles en el camp de visualitzacio
	{
		if (visibility[i]==1)
		{
			ind=1; // detectem un obstacle que veiem
			break;
		}
	}
	
	if (ind>0) // hi han obstacles que visualitzem
	{
		// treiem un vector dels index de les estacions de forma que indiqui com es ordenen de forma creixent segons llunyania
		vector<int> index(station.size()); // vector que contindra els indexs de les estacions ordenats de forma creixent, segons llunyania d'aquestes
		size_t n(0);
		generate(begin(index), end(index), [&]{ return n++; });
		sort(begin(index),end(index),[&](int i1, int i2) { return station[i1] < station[i2]; } );
		
		for (int i=0; i<num_static_obs; i=i+1)
		{
			int j=index[i];
			if (visibility[j] == 1)
			{
				obstacle.push_back(list[j]); // afegim a la llista de obstacles propers el obstacle trobat
				obs_station.push_back(station[j]); // construim el vector de 'stations' associat als obstacles visibles
				no_obstacle=1; //retornem que hi ha algun obstacle que visualitzem
			}
		}
	}
}

void C_StaticObs::get_all_closest_obs_interval(float target_i_ini, float target_i_fin, vector <C_Obstacle> &obstacle, vector <float> &obs_station, int &no_obstacle)
{
	no_obstacle=0; // continuara valent 0 si no en detecta cap obstacle
	int ind=0; // indicador per saber si hi han obstacles que veiem
	
	for (int i=0; i<(float)(visibility.size()); i=i+1) // mirem primer si tenim algun obstacles en el camp de visualitzacio
	{
		if (visibility[i]==1)
		{
			ind=1; // detectem un obstacle que veiem
			break;
		}
	}
	
	if (ind>0) // hi han obstacles que visualitzem
	{
		// treiem un vector dels index de les estacions de forma que indiqui com es ordenen de forma creixent segons llunyania
		vector<int> index(station.size()); // vector que contindra els indexs de les estacions ordenats de forma creixent, segons llunyania d'aquestes
		size_t n(0);
		generate(begin(index), end(index), [&]{ return n++; });
		sort(begin(index),end(index),[&](int i1, int i2) { return station[i1] < station[i2]; } );
		
		for (int i=0; i<num_static_obs; i=i+1)
		{
			int j=index[i];
			if ((visibility[j] == 1) && (station[j] > target_i_ini) && (station[j] <= target_i_fin))
			{
				obstacle.push_back(list[j]); // afegim a la llista de obstacles propers el obstacle trobat
				obs_station.push_back(station[j]); // construim el vector de 'stations' associat als obstacles visibles
				no_obstacle=1; //retornem que hi ha algun obstacle que visualitzem
			}
		}
	}
}

int C_StaticObs::get_num_obs(void)
{
		return num_static_obs;
}

C_Obstacle C_StaticObs::get_obs(int id, int &no_obstacle)
{
	no_obstacle=0; //d'entrada posem a 0 a questa variable per indicar que no s'ha troabt obstacle
	
	for (int i=0; i<num_static_obs; i=i+1)
	{
		if (list[i].id==id)
		{
			no_obstacle=1;// per indicar que s'ha trobat obstacle amb aquell id
			return list[i];
			break;
		}
	}
	
	float pos[2];
	pos[0]=0;
	pos[1]=0;
	float r=0;
	int id2=0;
	
	C_Obstacle obs_inexistence (pos,r,id2);
	return obs_inexistence; // per quan no trobi cap obstacle amb el 'id' indicat, retornara un obstacle amb tots els seus parametres a 0
	
}


