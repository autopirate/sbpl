#include <iostream>
#include <random>
#include <sbpl/headers.h>
#include <math.h>

using namespace std;




int main()

{	
	std::default_random_engine generator;
    std::normal_distribution<double> pt1x(5,1.0);

    for(int i = 0; i <20; i++)
    {
    	cout<<roundf(pt1x(generator)*100)/100<< endl;;
    }
    

}