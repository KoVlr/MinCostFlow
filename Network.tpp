#include "Network.h"
#include "Graph.h"


template<typename T>
Network<T>::Network()
{}


template<typename T>
Network<T>::Network(Graph<T>& Capacity) : Capacity(Capacity)
{
	for(const auto& edge : Capacity.get_edges())
	{
		Flow.get_edges()[{edge.first.first, edge.first.second}] = 0;
	}
}


template<typename T>
Graph<T> Network<T>::build_increment() const
{
	Graph<T> incrementCp;
	for (const auto &edge : Capacity.get_edges())
	{
		int u = edge.first.first;
		int v = edge.first.second;
		T capacity = edge.second;
		if (Flow.get_edges().at({u,v}) < capacity)
		{
			incrementCp.get_edges()[{u,v}] = capacity - Flow.get_edges().at({u,v});
		}
		if (Flow.get_edges().at({u,v}) > 0)
		{
			incrementCp.get_edges()[{v,u}] = Flow.get_edges().at({u,v});
		}
	}
	return incrementCp;
}


template<typename T>
void Network<T>::make_max_flow()
{
	int N = Capacity.number_of_vertexes();
	for (const auto &edge : Capacity.get_edges())
	{
		int u = edge.first.first;
		int v = edge.first.second;
		Flow.get_edges()[{u,v}] = 0;
	}

	while (true)
	{
		Graph<T> incrementCp = build_increment();
		std::forward_list<int> path = incrementCp.bread_first_search_path(0, N-1);
		if (path.empty()) break;

		//pop first two path items to initialize delta
		int path_first_elem = path.front();
		path.pop_front();
		int path_second_elem = path.front();
		path.pop_front();
		T delta = incrementCp.get_edges()[{path_first_elem, path_second_elem}];
		//loop over the remaining pairs of vertexes of the path
		int u = path_second_elem;
		for (const auto &v : path)
		{
			if(incrementCp.get_edges()[{u,v}] < delta)
			{
				delta = incrementCp.get_edges()[{u,v}];
			}
			u = v;
		}
		path.push_front(path_second_elem);

		u = path_first_elem;
		for (const auto &v : path)
		{
			if(Capacity.get_edges().find({u,v}) != Capacity.get_edges().end())
			{
				Flow.get_edges()[{u,v}] += delta;
			}
			else
			{
				Flow.get_edges()[{v,u}] -= delta;
			}
			u = v;
		}
	}
}


template<typename T>
Graph<T> Network<T>::get_positive_flow() const
{
	Graph<T> Positive_flow;
	for (const auto &edge : Flow.get_edges())
	{
		T flow = edge.second;
		if (flow > 0)
		{
			int u = edge.first.first;
			int v = edge.first.second;
			Positive_flow.get_edges()[{u,v}] = flow;
		}
	}
	return Positive_flow;
}


template<typename T>
T Network<T>::calculate_flow_value() const
{
	T flow_value = 0;
	int N = Capacity.number_of_vertexes();
	for (int v = 0; v < N; v++)
	{
		if (Flow.get_edges().find({0,v}) != Flow.get_edges().end())
		{
			flow_value += Flow.get_edges().at({0,v});
		}
	}
	return flow_value;
}


template<typename T>
void Network<T>::make_flow(T flow_value)
{
	int N = Capacity.number_of_vertexes();
	make_max_flow();
	T max_flow_value = calculate_flow_value();
	T over_flow_value = max_flow_value - flow_value;
	while(over_flow_value > 0)
	{
		std::forward_list<int> path = get_positive_flow().bread_first_search_path(0, N-1);
		int path_start = path.front();
		path.pop_front();
		T delta = over_flow_value;
		int u = path_start;
		for (const auto &v : path)
		{
			if (Flow.get_edges()[{u,v}] < delta)
			{
				delta = Flow.get_edges()[{u,v}];
			}
			u = v;
		}

		u = path_start;
		for (const auto &v : path)
		{
			Flow.get_edges()[{u,v}] -= delta;
			u = v;
		}
		over_flow_value -= delta;
	}
}


template<typename T>
const Graph<T>& Network<T>::get_Capacity() const
{
	return Capacity;
}


template<typename T>
Graph<T>& Network<T>::get_Flow()
{
	return Flow;
}


template<typename T>
const Graph<T>& Network<T>::get_Flow() const
{
	return Flow;
}