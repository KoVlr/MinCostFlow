#include "Graph.h"
#include <fstream>
#include <sstream>
#include <string>
#include <list>
#include <forward_list>
#include <queue>
#include <map>
#include <vector>
#include <limits>


template<typename T>
Graph<T>::Graph()
{}


template<typename T>
Graph<T>::Graph(std::string filename)
{
	std::ifstream file(filename);
	char delimiter = ',';
	std::string line;
	getline(file, line); //reading csv file header
	while(getline(file, line))
	{
		std::stringstream stream_line(line);
		std::string u;
		std::string v;
		getline(stream_line, u, delimiter);
		getline(stream_line, v, delimiter);
		//only the edge weight remains in the stream_line
		//and it is written to std::map
		stream_line >> edges[{stoi(u), stoi(v)}];
	}
	file.close();
}


template<typename T>
std::map<std::pair<int, int>, T>& Graph<T>::get_edges()
{
	return edges;
}


template<typename T>
const std::map<std::pair<int, int>, T>& Graph<T>::get_edges() const
{
	return edges;
}


template<typename T>
std::string Graph<T>::get_str() const
{
	std::stringstream output;
	output << "u,v,weight" << '\n';
	for (const auto &edge : edges)
	{
		output << edge.first.first << ',' << edge.first.second << ',' << edge.second << '\n';
	}
	return output.str();
}


template<typename T>
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


template<typename T>
std::vector<std::vector<T>>  Graph<T>::get_matrix(T diagonal_elem, T non_edge_elem) const
{
	int N = number_of_vertexes();

	std::vector<std::vector<T>> graph_matrix(N);
	
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


template<typename T>
std::pair<std::vector<std::vector<T>>, std::vector<std::vector<int>>> Graph<T>::floyd_warshall() const
{
	T infinity = std::numeric_limits<T>::max();
	int N = number_of_vertexes();
	std::vector<std::vector<T>> ShortestPath = get_matrix(0, infinity);
	std::vector<std::vector<int>> Previous_on_SP(N);
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
				T path_through_i = ShortestPath[u][i] + ShortestPath[i][v];
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
	return std::pair<std::vector<std::vector<T>>, std::vector<std::vector<int>>>(ShortestPath, Previous_on_SP);
}


template<typename T>
std::list<int> Graph<T>::find_negative_cycle() const
{
	std::list<int> negative_cycle;
	try
	{
		floyd_warshall();
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


template<typename T>
std::forward_list<int> Graph<T>::bread_first_search_path(int from, int to) const
{
	int N = number_of_vertexes();
	std::queue<int> to_visit;
	std::vector<bool> used(N, false);
	std::vector<int> previous_on_path(N, -1);
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
					to_visit = std::queue<int>();
					break;
				}
				to_visit.push(v);
			}
		}
	}
	std::forward_list<int> path;
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


negative_cycle_exception::negative_cycle_exception
(
	int v_in_negative_cycle,
	std::vector<std::vector<int>> Previous_on_SP
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


const std::vector<std::vector<int>>& negative_cycle_exception::get_Previous_on_SP() const
{
	return Previous_on_SP;
}