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

template<class T> using graph = map<pair<int, int>, T>;

template<class T>
void print_graph(const graph<T> &G)
{
	cout << 'u' << '\t' << 'v' << '\t' << "weight" << '\n';
	for (const auto &edge : G)
	{
		cout << edge.first.first << '\t' << edge.first.second << '\t' << edge.second << '\n';
	}
}


template <class T>
int vertexes_number(const graph<T> &G)
{
	int max_v = -1;
	for (const auto &edge : G)
	{
		if (edge.first.first > max_v) max_v = edge.first.first;
		if (edge.first.second > max_v) max_v = edge.first.second;
	}
	return max_v + 1;
}


template<class T>
vector<vector<T>> get_graph_matrix(const graph<T> &G, T diagonal_elem, T non_edge_elem)
{
	int N = vertexes_number(G);

	vector<vector<T>> graph_matrix(N);
	
	for (int u = 0; u < N; u++)
	{
		for (int v = 0; v < N; v++)
		{
			if (u==v) graph_matrix[u].push_back(diagonal_elem);
			else graph_matrix[u].push_back(non_edge_elem);
		}
	}

	for (const auto &edge : G)
	{
		int u = edge.first.first;
		int v = edge.first.second;
		T weight = edge.second;
		graph_matrix[u][v] = weight;
	}

	return graph_matrix;
}


int floyd_warshall(vector<vector<double>> &ShortestPath, vector<vector<int>> &Previous_on_SP)
{
	int N = ShortestPath.size();

	//initialization of matrix of previous vertexes on shortest path
	Previous_on_SP.clear();
	Previous_on_SP.resize(N);
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
			for (int v = 0; v < N; v++) {
				double path_through_i = ShortestPath[u][i] + ShortestPath[i][v];
				if (path_through_i < ShortestPath[u][v])
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
				return j;
			}
		}
	}
	return -1;
}


list<int> find_negative_cycle(const graph<double> &G)
{
	vector<vector<double>> ShortestPath = get_graph_matrix<double>(G, 0, inf);
	vector<vector<int>> Previous_on_SP;
	int v_in_negative_cycle = floyd_warshall(ShortestPath, Previous_on_SP);
	list<int> negative_cycle;
	if (v_in_negative_cycle != -1)
	{
		int u = v_in_negative_cycle;
		do
		{
			negative_cycle.push_front(u);
			u = Previous_on_SP[v_in_negative_cycle][u];
		} while(u != v_in_negative_cycle);
	}
	return negative_cycle;
}

template <class T>
forward_list<int> bread_first_search_path(const graph<T> &G, int from, int to)
{
	int N = vertexes_number(G);
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
			if(used[v] == false && G.find({u,v}) != G.end())
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

template <class T>
graph<T> build_increment(const graph<T> &Capacity, const graph<T> &Flow)
{
	graph<T> incrementCp;
	for (const auto &edge : Capacity)
	{
		int u = edge.first.first;
		int v = edge.first.second;
		T capacity = edge.second;
		if (Flow.at({u,v}) < capacity)
		{
			incrementCp[{u,v}] = capacity - Flow.at({u,v});
		}
		if (Flow.at({u,v}) > 0)
		{
			incrementCp[{v,u}] = Flow.at({u,v});
		}
	}
	return incrementCp;
}


template <class T>
graph<T> find_max_flow(const graph<T> &Capacity)
{
	graph<T> Flow;
	int N = vertexes_number(Capacity);
	for (const auto &edge : Capacity)
	{
		int u = edge.first.first;
		int v = edge.first.second;
		Flow[{u,v}] = 0;
	}

	while (true)
	{
		graph<T> incrementCp = build_increment<T>(Capacity, Flow);
		forward_list<int> path = bread_first_search_path<T>(incrementCp, 0, N-1);
		if (path.empty()) break;

		//pop first two path items to initialize delta
		int path_first_elem = path.front();
		path.pop_front();
		int path_second_elem = path.front();
		path.pop_front();
		T delta = incrementCp[{path_first_elem, path_second_elem}];
		//loop over the remaining pairs of vertexes of the path
		int u = path_second_elem;
		for (const auto &v : path)
		{
			if(incrementCp[{u,v}] < delta)
			{
				delta = incrementCp[{u,v}];
			}
			u = v;
		}
		path.push_front(path_second_elem);

		u = path_first_elem;
		for (const auto &v : path)
		{
			if(Capacity.find({u,v}) != Capacity.end())
			{
				Flow[{u,v}] += delta;
			}
			else
			{
				Flow[{v,u}] -= delta;
			}
			u = v;
		}
	}
	return Flow;
}

template<class T>
graph<T> get_positive_flow(const graph<T> &Flow)
{
	graph<T> Positive_flow;
	for (const auto &edge : Flow)
	{
		T flow = edge.second;
		if (flow > 0)
		{
			int u = edge.first.first;
			int v = edge.first.second;
			Positive_flow[{u,v}] = flow;
		}
	}
	return Positive_flow;
}

template<class T>
graph<T> find_flow(const graph<T> &Capacity, T flow_value)
{
	graph<T> Flow = find_max_flow<T>(Capacity);
	T max_flow_value = 0;
	int N = vertexes_number(Capacity);
	for (int v = 0; v < N; v++)
	{
		if (Flow.find({0,v}) != Flow.end())
		{
			max_flow_value += Flow[{0,v}];
		}
	}

	T over_flow_value = max_flow_value - flow_value;
	while(over_flow_value > 0)
	{
		forward_list<int> path = bread_first_search_path<T>(get_positive_flow<T>(Flow), 0, N-1);
		int path_start = path.front();
		path.pop_front();
		T delta = over_flow_value;
		int u = path_start;
		for (const auto &v : path)
		{
			if (Flow[{u,v}] < delta)
			{
				delta = Flow[{u,v}];
			}
			u = v;
		}

		u = path_start;
		for (const auto &v : path)
		{
			Flow[{u,v}] -= delta;
			u = v;
		}
		over_flow_value -= delta;
	}
	return Flow;
}


graph<double> find_min_cost_flow(const graph<double> &Capacity, const graph<double> &Cost, double flow_value)
{
	graph<double> Flow = find_flow<double>(Capacity, flow_value);

	while(true)
	{
		graph<double> incrementCp = build_increment<double>(Capacity, Flow);
		graph<double> incrementCost;

		//build increment cost graph
		for(const auto &edge : incrementCp)
		{
			int u = edge.first.first;
			int v = edge.first.second;
			if(Capacity.find({u,v}) != Capacity.end())
			{
				incrementCost[{u,v}] = Cost.at({u,v});
			}
			else
			{
				incrementCost[{u,v}] = -Cost.at({v,u});
			}
		}

		list<int> negative_cycle = list<int>(find_negative_cycle(incrementCost));
		if (negative_cycle.empty()) break;

		//find delta
		double delta = inf;
		int u = negative_cycle.back();
		for (const auto &v : negative_cycle)
		{
			if (incrementCp[{u,v}] < delta) 
			{
				delta = incrementCp[{u,v}];
			}
			u = v;
		}

		//change Flow
		u = negative_cycle.back();
		for (const auto &v : negative_cycle)
		{
			if(incrementCost[{u,v}] < 0) Flow[{v,u}] -= delta;
			else if (incrementCost[{u,v}] > 0) Flow[{u,v}] += delta;
			u = v;
		}
	}
	return Flow;
}


//reads graph from file and represents it as map that associates edges with weights
//the file must be in csv format
//columns: the first vertex of the edge, the second vertex of the edge, weight
template<class T> //T - type of graph weights
graph<T> read_graph(string filename)
{
	ifstream file(filename);
	char delimiter = ',';
	graph<T> G;
	string line;
	while(getline(file, line))
	{
		stringstream stream_line(line);
		string u;
		string v;
		getline(stream_line, u, delimiter);
		getline(stream_line, v, delimiter);
		//only the edge weight remains in the stream_line
		//and it is written to map
		stream_line >> G[{stoi(u), stoi(v)}];
	}
	file.close();
	return G;
}

double calculate_cost(const graph<double> &Cost, const graph<double> &Flow)
{
	double cost = 0;
	for (const auto &edge : Flow)
	{
		int u = edge.first.first;
		int v = edge.first.second;
		double flow = edge.second;
		cost += flow * Cost.at({u,v});
	}
	return cost;
}

int main()
{
	graph<double> Capacity = read_graph<double>("Capacity.csv");
	graph<double> Cost = read_graph<double>("Cost.csv");
	graph<double> Flow = find_min_cost_flow(Capacity, Cost, 20);
	print_graph(Flow);
	cout << calculate_cost(Cost, Flow);
	return 0;
}