#include "Graph.h"
#include "CostedNetwork.h"

#include <iostream>

int main()
{
	Graph<double> Capacity("Capacity.csv");
	Graph<double> Cost("Cost.csv");
	CostedNetwork<double> Net(Capacity, Cost);
	Net.make_flow(20);
	Net.minimize_cost();
	std::cout << Net.get_Flow().get_str();
	std::cout << '\n' << Net.calculate_cost();
	return 0;
}