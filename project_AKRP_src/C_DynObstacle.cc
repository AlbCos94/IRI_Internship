#include "C_DynObstacle.h" 


using namespace std; 

//constructor 
C_DynObstacle::C_DynObstacle (float t_01, float delta_t1, float v1, float r1, vector <vector<float> > path1, float dist, float res, int id1, int lane1)
{
	t_0=round(t_01/delta_t1)*delta_t1; // instant de temsp inicial en el que es comença a moure el vehicle
	delta_t=delta_t1; 
	v=v1; // velocitat a la que va el vehicle (cte)
	r=r1; //rado circunferencia defineix el vehicle
	path=path1;
	
	float max_index= (path[0].size()-1); // ultim 'numero de punt' que seguira el vehicle (pot seguir el vehicle, ja que de l'altre manera es queda sense carretera) 
	
	T=((dist/v + t_0)/delta_t)*delta_t; //round( (dist/v + t_0)/delta_t )*delta_t; // moment de temps en que es deixara de moure i haurà recorregut la distancia 'dist'
	
	//calcul vector de temps
	vector<float> time; // construim vector de temps desde el moment el cotxe es comença a moure fins que es para (perque ha realitzat la distancia indicada 'dist')
	for (float i=t_0; i<T; i=i+delta_t) //(el vector temps nomes inclou mentre es mou el cotxe !!)
	{
		time.push_back(i);
	}
	time.push_back(T);
	
	vector<float> positions_indexes (time.size(),0); // fem un vector de la mateixa magnitud que el vector de temps, per tal d'indicar en quin número de punt de la carretera ('station') ens trobem a cada instant de temps
	vector <vector<float> > poses; // fem un altre vector, de dos vectors, que contingui cada un les coordenades X i les Y ?¿?
	vector<float> posesX (time.size(),0);
	vector<float> posesY (time.size(),0);
	poses.push_back(posesX);
	poses.push_back(posesY);
	float t; // vector temps generic (desde que comença el algoritme)
	
	for (float i=0; i<=time.size(); i=i+1)
	{
		t=time[i]-t_0+delta_t;
		positions_indexes[i]=(round(v*t/res)-1); //calcul del numero del 'punt de la carretera', on 'res' es la resolucio (distancia entre 'stations' (punts en que es discretitza el carril))--> el '-1' es perque nosaltres iniciem la indexacio en el '0'
		if (positions_indexes[i] < 0)
		{
			positions_indexes[i]=0;
		}
		else if (positions_indexes[i] > max_index)
		{
			positions_indexes[i]= max_index;
		}					
		// relacionem el cada 'numero de punt de la trajectia' amb les coordenades X i Y que li corresponen
		poses[0][i]=path[0][positions_indexes[i]]; // trec la coordenada X que correspon a la coordenada del carril en aquell 'numero de punt'
		poses[1][i]=path[1][positions_indexes[i]]; // trec la coordenada Y que correspon a la coordenada del carril en aquell 'numero de punt'
	}
	//construim el vector 'position' --> format per un vector , de dos altres vectors amb les coordenades X i Y; un altre vector format pels instantsd de temps en que s'arriven a aquelles posicion i per ultim un vector corresponen als indexs del carril (els 'numero de punts')  
	positions.push_back(poses[0]); // introduim el vector de posicion de coordenades X
	positions.push_back(poses[1]); // introduim el vector de posicion de coordenades Y
	positions.push_back(time); // instants de temps en que arrivara a les dierents posicion (temps propi del vehicle desde que es comença a moure) 
	positions.push_back(positions_indexes); // indexos del carril ('número de punts')
	//positions.push_back(T);

	// vector de orientacions del vehicle
	//float n positions[0].size(); // NOMES SEMBLA SER QUE ES PER FER EL PLOT --> NO CAL POSARLO
	
	id=id1;
	visible=0; // al inici no veiem ( no es mou) el vehicle 
	lane=lane1;
	
}



//escrivim les funcions que estan definides com a membres de funcio de la classe C_DynObstacle

void C_DynObstacle::update_visibility(float current_t)
{
	float t= round(current_t/delta_t)*delta_t; // 't' ha de ser multiple de delta_t
	
	if ((t>=positions[2][0]) && (t <= positions[2].back())) // periode de temps mentre el qual el vehicle s'esta movent [t_0 , T]
	{
		visible=1; // periode en que es mou el vehicle, fem que el vehicle sigui visible
	}
	else
	{
		visible=0;
	}
}


