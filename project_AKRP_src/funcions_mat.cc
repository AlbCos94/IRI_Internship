// cabecera con funciones
#include "funcions_mat.h" 

float polyval(float p[], int n, float x) // n--> ordre del polinomi 'p', p--> array amb els coeficients del polinomi (p(t)=a*(t^3)+b*(t^2)+c*t+d(començant pel de grau mes elevat) amb p=[d,c,b,a]) x--> es el valor que vull evaluar
{										
    float px=0.0; 
    
    for (int i=0; i<(n+1); i++) //per a cada valor de exponent (para e=0 a 5)
    {
       //px=5;
       //px=px+1;
       px = px + (p[i]*pow(x,i)); 
    }
    
    return px;
    
 }    


void polyder(float p[], int n, float dp[]) // p --> array de coeficientes del polinomio, n --> es el orden del polinomio , dp --> array de los coeficientes de la derivada del polinomio (output)
{ 
    //dp={0};
    for(int i=0; i<n; i++) 
    {
        dp[i] = p[i+1] * (i+1.0); // le sumo 
    }
    
}

void polyint(float p[], int n, float ip[], float x_0) // p --> array de coeficientes del polinomio, n --> es el orden del polinomio , ip --> array de los coeficientes de la integral del polinomio (output), x_0 --> es la constante de la integal (termino independiente)
{ 
    //dp={0};
    for(int i=0; i<(n+2); i++) 
    {
		if (i==0)
		{
			ip[i]=x_0;
		}
		else
		{
			ip[i] = ((float)p[i-1]) * (1.0/(float)i); // le sumo 
		}
    
    }
}

void findpeaks(vector<float> values, vector<float> &peaks, vector<float> &locations)
{
	int count = 1; // numero de valors analitzats
	int noOfPeaks = 0; //numero de peaks
	float num_values=values.size(); //numero de valors que tenim en el vector
  
    for(int i = 0; i<num_values; i++)
    {
		if ( (count == 1) && (values[i] > values[i+1]) )
		{
			locations.push_back(i); 
			peaks.push_back(values[i]);
			noOfPeaks++;
			count++;
		}
		
		else if((count == num_values) && (values[i]>values[i-1]) )
		{
			locations.push_back(i); 
			peaks.push_back(values[i]);
			noOfPeaks++;
			count++;
		}
		
		else if ( (values[i] >= values[i+1]) && (values[i] > values[i-1]) )
		{
			locations.push_back(i); 
			peaks.push_back(values[i]);
			noOfPeaks++;
			count++;
		}
    }
}


void abs(vector<float> &values)
{
	for (int i=0; i<(float)values.size(); i++) 
	{
		if(values[i]<0)
		{
			values[i]*=-1;
		}
	}
}


void round_values(vector<float> &values)
{
	for (int i=0; i<(float)values.size(); i++) 
	{
		
		values[i]=round(values[i]);
		
	}
}



float constrain_angle(float angle, float range)
{
	angle=fmod((angle+360-range), 360.0);
	if (angle<0)
	{
		angle=angle+360;
	}
	return (angle-360+range);
	
}

float sum(vector<float> values)
{
	float suma=0;
	for (int i=0; i<(float)values.size(); i++) 
	{
		suma=suma+values[i];
	}
	return suma;
}


int sum_int(vector<int> values)
{
	int suma=0;
	for (int i=0; i<(float)values.size(); i++) 
	{
		suma=suma+values[i];
	}
	return suma;
}

float max_value(vector<float> values)
{
	float value_max=-1000;
	for (int i=0; i<(float)values.size(); i++) 
	{
		if (value_max<values[i])
		{
			value_max=values[i];
		}
	}
	return value_max;
			
}

float min_value(vector<float> values)
{
	float value_min=10000000;
	for (int i=0; i<(float)values.size(); i++) 
	{
		if (value_min>values[i])
		{
			value_min=values[i];
		}
	}
	return value_min;
			
}


