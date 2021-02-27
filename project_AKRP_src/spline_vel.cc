//header de la funcio
#include "spline_vel.h" 


using namespace std;

#define orderV 3 // ordre de la Spline del perfil de velocitat (funcio feta per Spline d'ordre 3)
#define tempsV0 20 // minim temps de calcul per quan ens exigeixen una estacio pero la velocitat final es 0 


//funcio de generacio de perfils de velocitats sense acceleració inicial 
void spline_3rdo_vel(float a_acc,float a_fre,float v_0,float v_f,float x_0, float t_0, float delta_t, float min_end_pos, std::vector<float>& Punts, std::vector<float>& Vels, std::vector<float>& Accels, std::vector<float>& temps)
{
	
	float dif = v_f-v_0; 
	
	if (dif*10==0) // cas en que les velocitats inicial i final son iguals --> perfil de 'vp' cte (una recta)
	{
		//equacins de moviment
		float v_t= v_0; // v(t)=vf=vo
		float a_t= 0;  // a(t)=0 
		float x_t[2]; // x(t)=v·t+Xo
		x_t[0]=x_0;
		x_t[1]=v_t;
		
		float T;
		
		if (min_end_pos!=0) // cas on tenim minima posicio final
		{
			T= min_end_pos/v_0;
		}
		else
		{
			T= delta_t*2.0;
		}
		
		if (v_0 < 0.1) // considerem que v_0=0
			T=20.0; // for example ¿?¿?¿?¿? --> tiempo minimo
			
		//calcul vector de temps
		std::vector<float> t; 
		for (float i=0; i<T; i=i+delta_t)
		{
			t.push_back(i);
		}
		//t.push_back(T);
						
		// Calcul dels valors d'espai (Punts), velocitats (Vels) i acceleracions (Accels) a cada instant
		for(int i=0; i<(float)t.size(); i++) 
		{
			Punts.push_back(polyval(x_t,1,t[i]));
			Vels.push_back(v_t);
			Accels.push_back(a_t);
			temps.push_back(t_0+t[i]); 
		}
		
				
		
	}
	
	else // v0 i vf diferents
	{
		float a_max;
		if (dif > 0)
		{
            a_max = a_acc; //Canvi de signe de l'acceleracio
        }
        else
        {
            a_max = -a_fre;
        }
        
        //coeficients equacions de moviment
        float b= 4.0*pow(a_max,2)/(3.0*dif);
        float a= (-pow(b,2))/(3.0*a_max);
        float c=0.0;
        float d= v_0;
        
        float v_t[orderV+1]; //array que conte els coeficients del polinomi de velocitat (num_coeficients=orderSpline+1)
		float a_t[orderV]; //array que conte els coeficients del polinomi de acceleracio (num_coeficients=orderSpline)
		float x_t[orderV+2]; //array que conte els coeficients del polinomi de posicio (num_coeficients=orderSpline+2)
        
        //equacions de moviment
		v_t[0]=d; v_t[1]=c; v_t[2]=b; v_t[3]=a; // v(t)=a*(t^3)+b*(t^2)+c*t+d
		polyder(v_t, orderV, a_t); // a(t)=3a*(t^2)+2b*(t^2)+c
		polyint(v_t, orderV, x_t, x_0); // x(t)=(a/4)*(t^4)+(b/3)*(t^3)+(c/2)*t^2+d*t+Xo
        
        float T = (3.0*dif)/(2.0*a_max);
        
        //calcul vector de temps
       	std::vector<float> t; 
		for (float i=0; i<T; i=i+delta_t)
		{
			t.push_back(i);
		}
        t.push_back(T);
     			
        // Calcul dels valors d'espai (Punts), velocitats (Vels) i acceleracions (Accels) a cada instant
		for(int i=0; i<(float)t.size(); i++) 
		{
			Punts.push_back(polyval(x_t,orderV+1,t[i]));
			Vels.push_back(polyval(v_t,orderV,t[i]));
			Accels.push_back(polyval(a_t,orderV-1,t[i]));
			
			temps.push_back(t_0+t[i]); 
		}
        
        if (min_end_pos!=0.0)
        {
			float end=Punts.size()-1; // definim el index de l'ultim element del vector temps
			
			
			
			if ((min_end_pos > Punts[end]) & (v_f>0)) // Si la posicio minima final es superior a la coordenada del ultim punt generat i v_f>0 --> afegirem un tram a velocitat constant, fins la estacio minima demanada
			{
				
				float dif_pos=min_end_pos - Punts[end]; // calculo l'espai entre la posicio de coordenada X minima y ultima posicio de coordenada X calculada 
				
								
				float v_t=v_f; // v(t)=vf
				float a_t=0; // a(t)=0
				float x_t[2]; // x(t)=v·t+Xo
				x_t[0]=Punts[end]; // agafem com a coordenada d'espai, la ultima coordenada espaial en X calculada
				x_t[1]=v_t;
				
				T = dif_pos/v_t;
				
				std::vector<float> t2; 
				for (float i=0; i<T; i=i+delta_t)
				{
					t2.push_back(i);
				}
				t2.push_back(T);
				
				// Calcul dels valors d'espai (Punts), velocitats (Vels) i acceleracions (Accels) a cada instant
				for(int i=0; i<(float)t2.size(); i++) 
				{
					Punts.push_back(polyval(x_t,1,t2[i]));
					Vels.push_back(v_t);
					Accels.push_back(a_t);
					temps.push_back(temps[end]+t2[i]); // elimnado para probar
				}
			}
			else if ((min_end_pos > Punts[end]) & (v_f==0)) //// Si la posicio minima final es superior a la coordenada del ultim punt generat i v_f=0 -->   afegirem un tram de velocitat nula cte
			{
						
				float v_t=v_f; // v(t)=vf
				float a_t=0; // a(t)=0 
				float x_t[2]; // x(t)=v·t+Xo
				x_t[0]=Punts[end];
				x_t[1]=v_t;
				
				
				T = tempsV0; //per exemple ?¿?
				
				std::vector<float> t2; 
				for (float i=0; i<T; i=i+delta_t)
				{
					t2.push_back(i);
				}
				t2.push_back(T);
				
			
				
				// Calcul dels valors d'espai (Punts), velocitats (Vels) i acceleracions (Accels) a cada instant
				for(int i=0; i<(float)t2.size(); i++) 
				{
					Punts.push_back(polyval(x_t,1,t2[i]));
					Vels.push_back(v_t);
					Accels.push_back(a_t);
					temps.push_back(temps[end]+t2[i]); // elimnado para probar 
					
				}
			}
		}	
			
	}
}

//**********************************************************************//
//funcio de generacio de perfils de velocitats amb acceleració inicial 
void spline_3rdo_vel_a0(float a_acc, float a_fre, float var_acc, float a_0, float v_0, float v_f, float x_0, float t_0, float delta_t, float min_end_pos, std::vector<float>& Punts, std::vector<float>& Vels, std::vector<float>& Accels, std::vector<float>& temps)
{
	
	float dif_v = v_f-v_0;
	
	if ((a_0*10)==0) // cas en que no hi ha acceleracio inicial o aquesta es molt petita
	{
		spline_3rdo_vel(a_acc, a_fre, v_0, v_f, x_0, t_0, delta_t, min_end_pos, Punts, Vels, Accels, temps);
	}
	
	else // cas on tenim acceleracio inicial
	{
		if (dif_v*10==0) // cas on velocitat inicial igual a la final (v_0=v_f)
		{
			float der_a_t[1];
			
			if (a_0>0)
			{
				der_a_t[0]=-var_acc;
			}
			else
			{
				der_a_t[0]=var_acc;			
			}
			
			int order_a=1;
			int order_v=2;
			int order_x=3;		
			
			float a_t[order_a+1]; 
			float v_t[order_v+1]; 
			float x_t[order_x+1]; 
			
			polyint(der_a_t, 0, a_t, a_0); 
			polyint(a_t, 1, v_t, v_0); 
			polyint(v_t, 2, x_t, x_0); 
			
			float T= -a_0/der_a_t[0];
			
			std::vector<float> t; 
			for (float i=0; i<T; i=i+delta_t)
			{
				t.push_back(i);
			}
			t.push_back(T);
			
					
			for(int i=0; i<(float)t.size(); i++) 
			{
				Punts.push_back(polyval(x_t,order_x,t[i]));
				Vels.push_back(polyval(v_t,order_v,t[i]));
				Accels.push_back(polyval(a_t,order_a,t[i]));
				temps.push_back(t_0+t[i]);
			}
			
			float end=t.size()-1; // index ultim element del vector
								
			spline_3rdo_vel(a_acc, a_fre, Vels[end], v_f, Punts[end], temps[end], delta_t, min_end_pos, Punts, Vels, Accels, temps); //// --> Spline case without a_0 / afegim la resta de valors que queden
			
		}
		else // cas on velocitat inicial diferent a la final (vf != v0)
		{
			if ( ((a_0 > 0) && (dif_v < 0)) || ((a_0 < 0) && (dif_v > 0)) ) //cas contradictiori --> el vehicle te un sentit de l'acceleracio inicial contrari al de la velocitat final (estic accelerant quan vull reduir la velocitat i idem pel contrari)
			{
				float der_a_t[1];
			
				if (a_0>0)
				{
					der_a_t[0]=-var_acc;
				}
				else
				{
					der_a_t[0]=var_acc;			
				}
				
				int order_a=1;
				int order_v=2;
				int order_x=3;		
			
				float a_t[order_a+1]; 
				float v_t[order_v+1]; 
				float x_t[order_x+1]; 
			
				polyint(der_a_t, 0, a_t, a_0); 
				polyint(a_t, 1, v_t, v_0); 
				polyint(v_t, 2, x_t, x_0); 
			
				float T= -a_0/der_a_t[0];
			
				std::vector<float> t; 
				for (float i=0; i<T; i=i+delta_t)
				{
					t.push_back(i);
				}
				t.push_back(T);
			
				for(int i=0; i<(float)t.size(); i++) 
				{
					Punts.push_back(polyval(x_t,order_x,t[i]));
					Vels.push_back(polyval(v_t,order_v,t[i]));
					Accels.push_back(polyval(a_t,order_a,t[i]));
					temps.push_back(t_0+t[i]);
				}
			
				float end=t.size()-1; // index ultim element del vector
			
				spline_3rdo_vel(a_acc, a_fre, Vels[end], v_f, Punts[end], temps[end], delta_t, min_end_pos, Punts, Vels, Accels, temps); // --> Spline case without a_0 /afegim la resta de valors que queden
				
				
			}
			else //cas favorable --> el vehicle te un sentit de l'acceleració igual al sentit de la velocitat final (estic accelerant quan vull augmentar la velocitat final o ja estic desaccelerant quan vull disminuir la velocitat final
			{
				float a_max; // acceleracio o desacceleracio maxima a la que s'arrivara
				
				if (dif_v > 0) // voldre continuar accelerant --> emprare acceleracion positives
				{
					a_max= a_acc; 
					if (a_max <= a_0) //l'acceleracio amb la que comencem ja es superior a la maxima marcada
					{
						a_max=a_0 + 0.01; // if not, the spline algorithm can't work, must to be a bit higher
					}
				}
				else // voldre continuar frenant --> emprare desacceleracion (acceleracions negatives)
				{
					a_max=-a_fre;
					if (a_max >= a_0) //la desacceleracio amb la que comencem, ja es inferior a la maxima marcada
					{
						a_max=a_0-0.01;
					}
				}
				
				float dif_a= a_0-a_max;
				
				//Calcul del temps total de la trajectoria a realitzar 'T' (temps en arribar a la velocitat dessitjada) --> resolucio eq de 2on ordre
				float A, B, C; // coeficients de la eq de 2on ordre
				A=((4*pow(a_0,2))/dif_a)-(3*a_0);
				B=(6*dif_v)-((12*(dif_v)*a_0)/dif_a);
				C=(9*pow(dif_v,2))/dif_a;
				
				float T1, T2, T; // solucions possibles dels temps de trajectoria (T1, T2) i temps que acabem agafant T (sempre el mes petit)
				T1=(-B+sqrt(pow(B,2)-4*A*C))/(2*A);
				T2=(-B-sqrt(pow(B,2)-4*A*C))/(2*A);
				
				if ((T1<0) || (T2<0))
				{
					T=max(T1,T2);
				}
				else
				{
					if (a_0>0) // Cas acceleracio positiva inicial
					{
						T=min(T1,T2);
					}
					else // Cas acceleracio negativa inicial
					{
						T=min(T1,T2);
					}
				}
				
				//coeficients equacions de moviment
				float a, b, c, d;
				b=(1/T)*(((3*dif_v)/T)-(2*a_0));
				a=pow(b,2)/(3*dif_a);
				c=a_0;
				d=v_0;
				
				
				float v_t[orderV+1]; 
				float a_t[orderV]; 
				float x_t[orderV+2]; 
        
				//equacions de moviment
				v_t[0]=d; v_t[1]=c; v_t[2]=b; v_t[3]=a; // v(t)=a*(t^3)+b*(t^2)+c*t+d
				polyder(v_t, orderV, a_t); // a(t)=3a*(t^2)+2b*(t^2)+c
				polyint(v_t, orderV, x_t, x_0); // x(t)=(a/4)*(t^4)+(b/3)*(t^3)+(c/2)*t^2+d*t+Xo
				
				//calcul vector de temps
				std::vector<float> t; 
				for (float i=0; i<T; i=i+delta_t)
				{
					t.push_back(i);
				}
				t.push_back(T);
        
				// Calcul dels valors d'espai (Punts), velocitats (Vels) i acceleracions (Accels) a cada instant
				for(int i=0; i<(float)t.size(); i++) 
				{
					Punts.push_back(polyval(x_t,orderV+1,t[i]));
					Vels.push_back(polyval(v_t,orderV,t[i]));
					Accels.push_back(polyval(a_t,orderV-1,t[i]));
			
					temps.push_back(t_0+t[i]); 
				}
				
				if (min_end_pos!=0.0) // exigim un posicio final determinada
				{
					float end=Punts.size()-1; // definim el index de l'ultim element del vector temps
					
					if ((min_end_pos > Punts[end]) & (v_f>0)) // Si la posicio minima final es superior a la coordenada del ultim punt generat i v_f>0 --> afegirem un tram a velocitat constant, fins la estacio minima demanada
					{
						float dif_pos = min_end_pos - Punts[end];
						
						float v_t=v_f; // v(t)=vf
						float a_t=0; // a(t)=0
						float x_t[2]; // x(t)=v·t+Xo
						x_t[0]=Punts[end]; // agafem com a coordenada d'espai, la ultima coordenada espaial en X calculada
						x_t[1]=v_t;
				
						T = dif_pos/v_t;
				
						std::vector<float> t2; 
						for (float i=0; i<T; i=i+delta_t)
						{
							t2.push_back(i);
						}
						t2.push_back(T);
				
						// Calcul dels valors d'espai (Punts), velocitats (Vels) i acceleracions (Accels) a cada instant
						for(int i=0; i<(float)t2.size(); i++) 
						{
							Punts.push_back(polyval(x_t,1,t2[i]));
							Vels.push_back(v_t);
							Accels.push_back(a_t);
							temps.push_back(temps[end]+t2[i]); 
						}
					}
					else if ((min_end_pos > Punts[end]) & (v_f==0)) //// Si la posicio minima final es superior a la coordenada del ultim punt generat i v_f=0 -->   afegirem un tram de velocitat nula cte
					{
						float v_t=v_f; // v(t)=vf
						float a_t=0; // a(t)=0 
						float x_t[2]; // x(t)=v·t+Xo
						x_t[0]=Punts[end];
						x_t[1]=v_t;
				
						T = tempsV0; //per exemple ?¿? calculara el punts punts en els proxims 20 segons
				
						std::vector<float> t2; 
						for (float i=0; i<T; i=i+delta_t)
						{
							t2.push_back(i);
						}
						t2.push_back(T);
				
						// Calcul dels valors d'espai (Punts), velocitats (Vels) i acceleracions (Accels) a cada instant
						for(int i=0; i<(float)t2.size(); i++) 
						{
							Punts.push_back(polyval(x_t,1,t2[i]));
							Vels.push_back(v_t);
							Accels.push_back(a_t);
							temps.push_back(temps[end]+t2[i]); 
						}
					} 
				}
			}
		}
	}
}



//**********************************************************************//
//funcio de generacio de perfils de velocitats (Amb la coordenada de l'estacio final fixada, emprada pel filtratge de candidats)
void spline_3rdo_velX(float v_0, float v_f, float x_0, float x_f, float t0, float delta_t, std::vector<float>& Punts, std::vector<float>& Vels, std::vector<float>& Accels, std::vector<float>& temps)
{
	float dif = v_f - v_0;
	
	if ((round(dif)*10)==0) // cas en que les velocitats inicial i final son iguals --> perfil de 'vp' cte (una recta)
	{
		//equacins de moviment
		float v_t= v_0; // v(t)=vf=vo
		float a_t= 0;  // a(t)=0 
		float x_t[2]; // x(t)=v·t+Xo
		x_t[1]=v_t;
		x_t[0]=x_0;
		
		//creacio vector de temps 't'
		float T = 5; // 5 s for examenple // ?¿ per que ¿? --> es treura el perfil pels proxims 5 segons?¿?
		std::vector<float> t; 
		for (float i=0; i<T; i=i+delta_t)
		{
			t.push_back(i);
		}
		
		
		// Calcul dels valors d'espai (Punts), velocitats (Vels) i acceleracions (Accels) a cada instant
		for(int i=0; i<(float)t.size(); i++) 
		{
			Punts.push_back(polyval(x_t,1,t[i]));
			Vels.push_back(v_t);
			Accels.push_back(a_t);
			temps.push_back(t0+t[i]);
		}
	
		//prova
		//for (int i=0; i<Punts.size(); i++)
		//{ 
			//temps.push_back(Punts[i]); 
		//}
		
	}
	
	else
	{
		float T=(2*x_f)/(v_0+v_f);
		
		//coeficients equacions de moviment
		float b = (3.0*dif)/pow(T,2.0);
		float a = -(2.0/3)*(b/T); // 2.0 i no 2 !! sino dona enter=0 !!
		float c=0;
		float d= v_0;
				
		float v_t[orderV+1]; //array que conte els coeficients del polinomi de velocitat (num_coeficients=orderSpline+1)
		float a_t[orderV]; //array que conte els coeficients del polinomi de acceleracio (num_coeficients=orderSpline)
		float x_t[orderV+2]; //array que conte els coeficients del polinomi de posicio (num_coeficients=orderSpline+2)
		
		//equacins de moviment
		v_t[0]=d; v_t[1]=c; v_t[2]=b; v_t[3]=a; // v(t)=a*(t^3)+b*(t^2)+c*t+d
		polyder(v_t, orderV, a_t); // a(t)=3a*(t^2)+2b*(t^2)+c
		polyint(v_t, orderV, x_t, x_0); // x(t)=(a/4)*(t^4)+(b/3)*(t^3)+(c/2)*t^2+d*t+Xo
							
		//creacio vector de temps 't'
		std::vector<float> t; 
		for (float i=0; i<T; i=i+delta_t)
		{
			t.push_back(i);
		}
		
			
		// Calcul dels valors d'espai (Punts), velocitats (Vels) i acceleracions (Accels) a cada instant
		for(int i=0; i<(float)t.size(); i++) 
		{
			Punts.push_back(polyval(x_t,orderV+1,t[i]));
			Vels.push_back(polyval(v_t,orderV,t[i]));
			Accels.push_back(polyval(a_t,orderV-1,t[i]));
			temps.push_back(t0+t[i]); 
		}
		
		
		
		
		////prova
		//for (int i=0; i<Punts.size(); i++)
		//{ 
			//temps.push_back(Punts[i]); 
		//}
		
		
	}
	
}
