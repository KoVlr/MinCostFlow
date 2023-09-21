#ifndef NETWORK
#define NETWORK

#include "Graph.h"

template<typename T>
class Network
{
	public:
		Network();
		Network(Graph<T>& Capacity);
		T calculate_flow_value() const;
		void make_max_flow();
		void make_flow(T flow_value);
		const Graph<T>& get_Capacity() const;
		Graph<T>& get_Flow();
		const Graph<T>& get_Flow() const;
	protected:
		const Graph<T> Capacity;
		Graph<T> Flow;
		Graph<T> build_increment() const;
		Graph<T> get_positive_flow() const; //return a graph that contains only positive edges of Flow
};


#include "Network.tpp"

#endif