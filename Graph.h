#ifndef GRAPH
#define GRAPH

#include <string>
#include <map>
#include <list>
#include <vector>
#include <forward_list>


template<typename T>
class Graph
{
	public:
		Graph();
		Graph(std::string filename);
		std::map<std::pair<int, int>, T>& get_edges();
		const std::map<std::pair<int, int>, T>& get_edges() const;
		std::string get_str() const; //listing graph edges as a string
		std::vector<std::vector<T>> get_matrix(T diagonal_elem, T non_edge_elem) const; //representation of a graph as a matrix
		std::pair<std::vector<std::vector<T>>, std::vector<std::vector<int>>> floyd_warshall() const;
		std::list<int> find_negative_cycle() const;
		std::forward_list<int> bread_first_search_path(int from, int to) const;
		int number_of_vertexes() const;
	private:
		std::map<std::pair<int, int>, T> edges;
};


//Exception thrown when trying to apply Floyd-Warshall algorithm to a graph with a negative cycle
class negative_cycle_exception : public std::exception
{
	public:
		negative_cycle_exception(int v_in_negative_cycle, std::vector<std::vector<int>> Previous_on_SP);
		const char* what() const noexcept override;
		int get_v_in_negative_cycle() const;
		const std::vector<std::vector<int>>& get_Previous_on_SP() const;
	private:
		int v_in_negative_cycle; //some vertex included in a negative cycle
		const std::vector<std::vector<int>> Previous_on_SP; //matrix of previous vertexes at the moment of detection a negative cycle
};


#include "Graph.tpp"

#endif