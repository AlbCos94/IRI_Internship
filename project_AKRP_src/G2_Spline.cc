//propies
#include "G2_Spline.h" 


using namespace std;

#define order 5 // ordre de la SpLine (funcio feta per Spline d'ordre 5)

void G2_Spline (float x_A, float y_A, float K_A, float theta_A, float x_B, float y_B, float K_B, float theta_B, int N, float resolution, std::vector<float>& Px, std::vector<float>& Py, float& path_dist, std::vector<float>& curvature)
{
	float n_1_2= sqrt(pow((x_B-x_A),2) + pow((y_B-y_A),2)); // initial approximation of the coeficient (for the path lenght)
	
	float num_points= n_1_2/resolution;
	
	float S;

	// Definicio dels coeficients de les Splines d'Ordre 5 (6 coeficients)	
		float x_u[order+1];
		float y_u[order+1];
				
	// Normalitzacio del conjunt de punts en que es discretitza la corva
	vector<float> u;
		
	for (float n=0; n<1; n=n+(1/num_points))
	{
		u.push_back(n);
	}
	if ( u.back()!=1 )
	{
		u.push_back(1); //last value always will be 1
	}	
	
	//Calcul iteratiu parametre n_1_2 
	for(int n=0; n<N; n++)
	{	
		float n_3_4=0; // per definicio
		
		float n1 = n_1_2;  // Mante l'orientacio inicial durant el maxim possible
		float n2 = n_1_2;  // Mante l'orientacio final durant el maxim possible
		float n3 = -n_3_4; // Modulacio de la curvatura a l'inici
		float n4 = n_3_4;  // Modulacio de la curvatura al final
		
		//Spline en X
		float x0, x1, x2, x3, x4, x5; // coeficients de la Spline en X
		
		x0 = x_A;
		x1 = n1*cos(theta_A);
		x2 = 0.5*(n3*cos(theta_A)-pow(n1,2)*K_A*sin(theta_A));
		x3 = 10*(x_B-x_A) - (6*n1+1.5*n3)*cos(theta_A) - (4*n2-0.5*n4)*cos(theta_B) + 1.5*pow(n1,2)*K_A*sin(theta_A) - 0.5*pow(n2,2)*K_B*sin(theta_B);
		x4 = -15*(x_B-x_A) + ((8*n1)+(3/2*n3))*cos(theta_A) + (7*n2-n4)*cos(theta_B)-3/2*pow(n1,2)*K_A*sin(theta_A) + pow(n2,2)*K_B*sin(theta_B);
		x5 = 6*(x_B-x_A) - (3*n1+1/2*n3)*cos(theta_A) - (3*n2-0.5*n4)*cos(theta_B)+ 1/2*pow(n1,2)*K_A*sin(theta_A) - 0.5*pow(n2,2)*K_B*sin(theta_B);
			
		//vector de los coeficientes de la Spline en X
		x_u[0]=x0; x_u[1]=x1; x_u[2]=x2; x_u[3]=x3; x_u[4]=x4; x_u[5]=x5; 
		
		//Spline en Y
		float y0, y1, y2, y3, y4, y5; // coeficients de la Spline en Y
			
		y0 = y_A;
		y1 = n1*sin(theta_A);
		y2 = 0.5*(n3*sin(theta_A)+pow(n1,2)*K_A*cos(theta_A));
		y3 = 10*(y_B-y_A) - (6*n1+3/2*n3)*sin(theta_A) - (4*n2-0.5*n4)*sin(theta_B)-3/2*pow(n1,2)*K_A*cos(theta_A) + 0.5*pow(n2,2)*K_B*cos(theta_B);
		y4 = -15*(y_B-y_A) + (8*n1+3/2*n3)*sin(theta_A) + (7*n2-n4)*sin(theta_B)+3/2*pow(n1,2)*K_A*cos(theta_A) - pow(n2,2)*K_B*cos(theta_B);
		y5 = 6*(y_B-y_A) - (3*n1+1/2*n3)*sin(theta_A) - (3*n2-0.5*n4)*sin(theta_B)-1/2*pow(n1,2)*K_A*cos(theta_A) + 0.5*pow(n2,2)*K_B*cos(theta_B);
			
		//vector de los coeficientes de la Spline en Y
		y_u[0]=y0; y_u[1]=y1; y_u[2]=y2; y_u[3]=y3; y_u[4]=y4; y_u[5]=y5; 
		
		
		// Derivades, calcul curvatura i longitud
		
		//en X
		float dx_u[order]; // la derivada te un grau menor al del polinomi
		polyder(x_u, order, dx_u); // calculo de los coeficientes de la derivada del polinomio en X
		vector<float> xdot_u;
		
		for(int i=0; i<(float)u.size(); i++) //for(int n=0; n<42; n++)
		{
			xdot_u.push_back(polyval(dx_u,(order-1),u[i])); // calculo del vector de valores de la derivada
		}
		
		//en Y
		float dy_u[order]; // la derivada te un grau menor al del polinomi
		polyder(y_u, order, dy_u); // calculo de los coeficientes de la derivada del polinomio en Y
		vector<float> ydot_u;
		
		for(int i=0; i<(float)u.size(); i++) //for(int n=0; n<42; n++)
		{
			ydot_u.push_back(polyval(dy_u,(order-1),u[i])); // calculo del vector de valores de la derivada
		}
		
		// Recalcul del parametre n_1_2
		
		S=0;
			
		for(int i=0; i<(float)u.size(); i++) //for(int n=0; n<42; n++)
		{
			S=S+sqrt(pow(xdot_u[i],2)+pow(ydot_u[i],2));
		}
		S=S/u.size();
		
		n_1_2=S;
		
	}	
	
	//Càlcul dels outputs
	
	// torno a calcular vector u amb el nou parametre n_1_2
	num_points= n_1_2/resolution;
	u.clear(); // borro l'anterior
	for (float n=0; n<1; n=n+(1/num_points))
	{
		u.push_back(n);
	}
	if ( u.back()!=1 )
	{
		u.push_back(1); //last value always will be 1
	}	 
	
	// --> Coordenades en X i Y de la Spline
	for(int i=0; i<(float)u.size(); i++) 
	{
		Px.push_back(polyval(x_u,order,u[i]));
		Py.push_back(polyval(y_u,order,u[i]));
	} 
	
	// --> Distancia del cami
	path_dist = S; 
	
	// calcul de la curvatura
	
	//vectors derivades primeres
	float dx_u[order];
	polyder(x_u, order, dx_u);
	vector<float> xdot_u;
	for(int i=0; i<(float)u.size(); i++) 
	{
		xdot_u.push_back(polyval(dx_u,(order-1),u[i])); 
	}
	
	float dy_u[order];
	polyder(y_u, order, dy_u);
	vector<float> ydot_u;
	for(int i=0; i<(float)u.size(); i++) 
	{
		ydot_u.push_back(polyval(dy_u,(order-1),u[i])); 
	}
	
	//vectors derivades segones
	float ddx_u[(order-1)]; // tindra un ordre menor la derivada 2a
	polyder(dx_u, (order-1), ddx_u);
	vector<float> xdotdot_u;
	for(int i=0; i<(float)u.size(); i++) 
	{
		xdotdot_u.push_back(polyval(ddx_u,(order-2),u[i])); 
	}
	
	float ddy_u[(order-1)];
	polyder(dy_u, (order-1), ddy_u);
	vector<float> ydotdot_u;
	for(int i=0; i<(float)u.size(); i++) 
	{
		ydotdot_u.push_back(polyval(ddy_u,(order-2),u[i])); 
	}
	
	// --> Vector de curvatures
	for(int i=0; i<(float)u.size(); i++) 
	{
		curvature.push_back( (xdot_u[i]*ydotdot_u[i]-ydot_u[i]*xdotdot_u[i])/pow(pow(xdot_u[i],2)+pow(ydot_u[i],2),1.5) ); 
	}
	
		
}


