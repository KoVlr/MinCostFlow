#ifndef COSTEDNETWORK
#define COSTEDNETWORK

#include "Graph.h"
#include "Network.h"

template<typename T>
class CostedNetwork: public Network<T>
{
	public:
        CostedNetwork();
		CostedNetwork(Graph<T>& Capcity, Graph<T>& Cost);
		T calculate_cost() const;
		void minimize_cost(); // minimizes cost without changing flow value
		const Graph<T>& get_Cost() const;
	private:
		const Graph<T> Cost;
		Graph<T> build_cost_increment() const;
};


#include "CostedNetwork.tpp"

#endif