#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <list>
#include <limits>
#include <forward_list>
#include <queue>
#include <map>
#include <vector>
#include <set>

using namespace std;

constexpr double inf = std::numeric_limits<double>::infinity();


//Exception thrown when trying to apply Floyd-Warshall algorithm to a graph with a negative cycle
class negative_cycle_exception : public exception
{
	public:
		negative_cycle_exception(int v_in_negative_cycle, vector<vector<int>> Previous_on_SP);
		const char* what() const noexcept override;
		int get_v_in_negative_cycle() const;
		const vector<vector<int>>& get_Previous_on_SP() const;
	private:
		int v_in_negative_cycle; //some vertex included in a negative cycle
		const vector<vector<int>> Previous_on_SP; //matrix of previous vertexes at the moment of detection a negative cycle
};

negative_cycle_exception::negative_cycle_exception
(
	int v_in_negative_cycle,
	vector<vector<int>> Previous_on_SP
): v_in_negative_cycle(v_in_negative_cycle), Previous_on_SP(Previous_on_SP)
{}

const char* negative_cycle_exception::what() const noexcept
{
	return "Graph contains a cycle with negative weight";
}

int negative_cycle_exception::get_v_in_negative_cycle() const
{
	return v_in_negative_cycle;
}

const vector<vector<int>>& negative_cycle_exception::get_Previous_on_SP() const
{
	return Previous_on_SP;
}


template<class T>
class Graph
{
	public:
		Graph(){}
		Graph(string filename);
		map<pair<int, int>, T>& get_edges();
		const map<pair<int, int>, T>& get_edges() const;
		string get_str() const; //listing graph edges as a string
		vector<vector<T>> get_matrix(T diagonal_elem, T non_edge_elem) const; //representation of a graph as a matrix
		pair<vector<vector<T>>, vector<vector<int>>> floyd_warshall(T infinity) const;
		list<int> find_negative_cycle(T infinity) const;
		forward_list<int> bread_first_search_path(int from, int to) const;
		int number_of_vertexes() const;
	private:
		map<pair<int, int>, T> edges;
};

template<class T>
class Network
{
	public:
		Network() {}
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

template<class T>
class CostedNetwork: public Network<T>
{
	public:
		CostedNetwork(Graph<T>& Capcity, Graph<T>& Cost);
		T calculate_cost() const;
		void minimize_cost(); // minimizes cost without changing flow value
		const Graph<T>& get_Cost() const;
	private:
		const Graph<T> Cost;
		Graph<T> build_cost_increment() const;
};

template<class T>
Graph<T>::Graph(string filename)
{
	ifstream file(filename);
	char delimiter = ',';
	string line;
	getline(file, line); //reading csv file header
	while(getline(file, line))
	{
		stringstream stream_line(line);
		string u;
		string v;
		getline(stream_line, u, delimiter);
		getline(stream_line, v, delimiter);
		//only the edge weight remains in the stream_line
		//and it is written to map
		stream_line >> edges[{stoi(u), stoi(v)}];
	}
	file.close();
}


template<class T>
map<pair<int, int>, T>& Graph<T>::get_edges()
{
	return edges;
}


template<class T>
const map<pair<int, int>, T>& Graph<T>::get_edges() const
{
	return edges;
}


template<class T>
string Graph<T>::get_str() const
{
	stringstream output;
	output << "u,v,weight" << '\n';
	for (const auto &edge : edges)
	{
		output << edge.first.first << ',' << edge.first.second << ',' << edge.second << '\n';
	}
	return output.str();
}


template<class T>
int Graph<T>::number_of_vertexes() const
{
	int max_v = -1;
	for (const auto &edge : edges)
	{
		if (edge.first.first > max_v) max_v = edge.first.first;
		if (edge.first.second > max_v) max_v = edge.first.second;
	}
	return max_v + 1;
}


template<class T>
vector<vector<T>>  Graph<T>::get_matrix(T diagonal_elem, T non_edge_elem) const
{
	int N = number_of_vertexes();

	vector<vector<T>> graph_matrix(N);
	
	for (int u = 0; u < N; u++)
	{
		for (int v = 0; v < N; v++)
		{
			if (u==v) graph_matrix[u].push_back(diagonal_elem);
			else graph_matrix[u].push_back(non_edge_elem);
		}
	}

	for (const auto &edge : edges)
	{
		int u = edge.first.first;
		int v = edge.first.second;
		T weight = edge.second;
		graph_matrix[u][v] = weight;
	}

	return graph_matrix;
}


template<class T>
pair<vector<vector<T>>, vector<vector<int>>> Graph<T>::floyd_warshall(T infinity) const
{
	int N = number_of_vertexes();
	vector<vector<T>> ShortestPath = get_matrix(0, infinity);
	vector<vector<int>> Previous_on_SP(N);
	//initialization of matrix of previous vertexes on shortest path
	for (int u = 0; u < N; u++)
	{
		for (int v = 0; v < N; v++)
		{
			Previous_on_SP[u].push_back(u);
		}
	}

	//find shortest paths
	for (int i = 0; i < N; i++)
	{
		for (int u = 0; u < N; u++)
		{
			if (ShortestPath[u][i] == infinity) continue;
			for (int v = 0; v < N; v++) {
				if(ShortestPath[i][v] == infinity) continue;
				double path_through_i = ShortestPath[u][i] + ShortestPath[i][v];
				if (ShortestPath[u][v] == infinity || path_through_i < ShortestPath[u][v])
				{
					ShortestPath[u][v] = path_through_i;
					Previous_on_SP[u][v] = Previous_on_SP[i][v];
				}
			}
		}

		//check negative cicle
		for (int j = 0; j < N; j++)
		{
			if (ShortestPath[j][j] < 0)
			{
				throw negative_cycle_exception(j, Previous_on_SP);
			}
		}
	}
	return pair<vector<vector<T>>, vector<vector<int>>>(ShortestPath, Previous_on_SP);
}


template<class T>
list<int> Graph<T>::find_negative_cycle(T infinity) const
{
	list<int> negative_cycle;
	try
	{
		floyd_warshall(infinity);
	}
	catch(negative_cycle_exception& e)
	{
		int u = e.get_v_in_negative_cycle();
		do
		{
			negative_cycle.push_front(u);
			u = e.get_Previous_on_SP()[e.get_v_in_negative_cycle()][u];
		} while(u != e.get_v_in_negative_cycle());
	}
	return negative_cycle;
}


template<class T>
forward_list<int> Graph<T>::bread_first_search_path(int from, int to) const
{
	int N = number_of_vertexes();
	queue<int> to_visit;
	vector<bool> used(N, false);
	vector<int> previous_on_path(N, -1);
	to_visit.push(from);
	used[from] = true;
	while(!to_visit.empty())
	{
		int u = to_visit.front();
		to_visit.pop();
		for (int v = 0; v < N; v++)
		{
			if(used[v] == false && edges.find({u,v}) != edges.end())
			{
				previous_on_path[v] = u;
				used[v] = true;
				if (v == to)
				{
					to_visit = queue<int>();
					break;
				}
				to_visit.push(v);
			}
		}
	}
	forward_list<int> path;
	if(used[to] == true)
	{
		int v = to;
		while (v != -1)
		{
			path.push_front(v);
			v = previous_on_path[v];
		}
	}
	return path;
}


template<class T>
Network<T>::Network(Graph<T>& Capacity) : Capacity(Capacity)
{
	for(const auto& edge : Capacity.get_edges())
	{
		Flow.get_edges()[{edge.first.first, edge.first.second}] = 0;
	}
}


template<class T>
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


template<class T>
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
		forward_list<int> path = incrementCp.bread_first_search_path(0, N-1);
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


template<class T>
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


template<class T>
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


template<class T>
void Network<T>::make_flow(T flow_value)
{
	int N = Capacity.number_of_vertexes();
	make_max_flow();
	T max_flow_value = calculate_flow_value();
	T over_flow_value = max_flow_value - flow_value;
	while(over_flow_value > 0)
	{
		forward_list<int> path = get_positive_flow().bread_first_search_path(0, N-1);
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


template<class T>
const Graph<T>& Network<T>::get_Capacity() const
{
	return Capacity;
}


template<class T>
Graph<T>& Network<T>::get_Flow()
{
	return Flow;
}


template<class T>
const Graph<T>& Network<T>::get_Flow() const
{
	return Flow;
}


template<class T>
CostedNetwork<T>::CostedNetwork(Graph<T>& Capacity, Graph<T>& Cost): Network<T>(Capacity), Cost(Cost)
{}


template<class T>
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


template<class T>
void CostedNetwork<T>::minimize_cost()
{
	while(true)
	{
		Graph<T> incrementCp = this->build_increment();
		Graph<T> incrementCost = build_cost_increment();

		list<int> negative_cycle = incrementCost.find_negative_cycle(inf);
		if (negative_cycle.empty()) break;

		//find delta
		T delta = inf;
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


template<class T>
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


template<class T>
const Graph<T>& CostedNetwork<T>::get_Cost() const
{
	return Cost;
}


int main()
{
	Graph<double> Capacity("Capacity.csv");
	Graph<double> Cost("Cost.csv");
	CostedNetwork<double> Net(Capacity, Cost);
	Net.make_flow(20);
	Net.minimize_cost();
	cout << Net.get_Flow().get_str();
	cout << '\n' << Net.calculate_cost();
	return 0;
}