#include "CostedNetwork.h"
#include "Graph.h"
#include <list>
#include <limits>


template<typename T>
CostedNetwork<T>::CostedNetwork()
{}


template<typename T>
CostedNetwork<T>::CostedNetwork(Graph<T>& Capacity, Graph<T>& Cost): Network<T>(Capacity), Cost(Cost)
{}


template<typename T>
Graph<T> CostedNetwork<T>::build_cost_increment() const
{
	Graph<T> incrementCost;
	for (const auto &edge : this->Capacity.get_edges())
	{
		int u = edge.first.first;
		int v = edge.first.second;
		T capacity = edge.second;
		if (this->Flow.get_edges().at({u,v}) < capacity)
		{
			incrementCost.get_edges()[{u,v}] = Cost.get_edges().at({u,v});
		}
		if (this->Flow.get_edges().at({u,v}) > 0)
		{
			incrementCost.get_edges()[{v,u}] = -Cost.get_edges().at({u,v});
		}
	}
	return incrementCost;
}


template<typename T>
void CostedNetwork<T>::minimize_cost()
{
	while(true)
	{
		Graph<T> incrementCp = this->build_increment();
		Graph<T> incrementCost = build_cost_increment();

		std::list<int> negative_cycle = incrementCost.find_negative_cycle();
		if (negative_cycle.empty()) break;

		//find delta
		T delta = std::numeric_limits<T>::max();
		int u = negative_cycle.back();
		for (const auto &v : negative_cycle)
		{
			if (incrementCp.get_edges()[{u,v}] < delta) 
			{
				delta = incrementCp.get_edges()[{u,v}];
			}
			u = v;
		}

		//change Flow
		u = negative_cycle.back();
		for (const auto &v : negative_cycle)
		{
			if(incrementCost.get_edges()[{u,v}] < 0) this->Flow.get_edges()[{v,u}] -= delta;
			else if (incrementCost.get_edges()[{u,v}] > 0) this->Flow.get_edges()[{u,v}] += delta;
			u = v;
		}
	}
}


template<typename T>
T CostedNetwork<T>::calculate_cost() const
{
	T cost = 0;
	for (const auto &edge : this->Flow.get_edges())
	{
		int u = edge.first.first;
		int v = edge.first.second;
		T flow = edge.second;
		cost += flow * Cost.get_edges().at({u,v});
	}
	return cost;
}


template<typename T>
const Graph<T>& CostedNetwork<T>::get_Cost() const
{
	return Cost;
}