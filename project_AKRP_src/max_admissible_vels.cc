//propies
#include "max_admissible_vels.h" 


using namespace std;


void max_admissible_vels (vector<float> LP_path_k, float resolution, float comfort_a_lat, float max_road_vel, float (&max_vel_restriction)[2])
{
	//establim valors inicials
	max_vel_restriction[0]=0;// la posicio per defecte sera de 0 metres
	max_vel_restriction[1]=max_road_vel; //establim com a maxima velocitat d'entrada, la permesa en la carretera
	
	//fem el valor absolut del vector de curvatures
	abs(LP_path_k);
	
	//busquem els maxims locals de les curvatures i els els punts en que es troben
	vector<float> peaks;
	vector<float> locations;
	
	findpeaks(LP_path_k, peaks, locations);
	
	float num_peaks=peaks.size();
	
	if (num_peaks>0)
	{
		vector<float> max_admissible_vels;
		for (int i=0; i<num_peaks; i++)
		{
			max_admissible_vels.push_back(sqrt(comfort_a_lat/peaks[i])); // contra mes gran sigui la curvatura (mes gran el pic), la velocitat maxima permesa en aquell punt sera menor (inversament proporcional curvatura a velocitat permesa)
		}
		vector<float> rounded;
		rounded=max_admissible_vels;
		round_values(rounded);  //arrodonim el vector de maximes velocitats admisibles
			
					
		// Ordenem de forma creixent les maximes velocitats admisibles i creem un vecto dels seus indexs de posicio (tambe en ordre creixent)
		vector<float> sorted;
		sorted=rounded;
		vector<int> indexs(sorted.size()); 
		size_t n(0);
		generate(std::begin(indexs), std::end(indexs), [&]{ return n++; });
		sort(std::begin(indexs),std::end(indexs),[&](int i1, int i2) { return sorted[i1] < sorted[i2]; } );
		
		sort(sorted.begin(), sorted.end()); //tambe ordenem el vector de velocitats de forma decreixent 
						
		//agafo com a maxima velocitat la minima adimssible
		max_vel_restriction[1] = sorted[0];
		
		if (max_vel_restriction[1] < max_road_vel) //si la maxima velocitat admisible  es menor a la de la propia carretera
		{
			max_vel_restriction[0] = (locations[indexs[0]]+1)*resolution; //indiquem els metres longitudinals de la carretera a partir de la qual s'haura de fer la restriccio de velocitat
		}
	}
}

