#ifndef __FUNCIONS_MAT_H_INCLUDED__
#define __FUNCIONS_MAT_H_INCLUDED__

#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

// Evaluar un valor en un polinomi de coeficients determinats
float polyval(float p[], int n, float x); // n--> ordre del polinomi, p--> array amb els coeficients del polinomi ((p(t)=a*(t^3)+b*(t^2)+c*t+d amb p=[d,c,b,a])), x--> es el valor que vull evaluar

// Realitza la dericada d'un polinomi d'un cert ordre 
void polyder(float p[], int n, float dp[]); // p --> array de coeficientes del polinomio, n --> es el orden del polinomio 'p', dp --> array de los coeficientes de la derivada del polinomio

// Realitza la integral d'un polinomi d'un cert ordre 
void polyint(float p[], int n, float ip[], float x_0); // // p --> array de coeficientes del polinomio, n --> es el orden del polinomio 'p', ip --> array de los coeficientes de la integral del polinomio (output), x_0 --> es la constante de la integal (termino independiente)

// Extreu els pics de un vector de valors donats i el vector de la posicio d'aquests
	//A local peak is defined as a data sample which is either larger than the two neighboring samples or is equal to Inf.
void findpeaks(vector<float> values, vector<float>& peaks, vector<float>& locations);

// Transforma en valor absolut tots els valors d'un cert vector
void abs(vector<float> &values);

// Arrodoneix tots els valors d'un cert vector al enter mes proper
void round_values(vector<float> &values);

// retorna el angle del rang donat
float constrain_angle(float angle, float range);

// retorna la suma dels valors d'un vector de valors float
float sum(vector<float> values);

// retorna la suma dels valors d'un vector de valors int
int sum_int(vector<int> values);

// retorna el maxim valor de un conjunt de valor de un vector
float max_value(vector<float> values);

// retorna el minim valor de un conjunt de valor de un vector
float min_value(vector<float> values);


#endif // !__FUNCIONS_MAT_H_INCLUDED__
