#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <list>
#include <limits>
#include <forward_list>
#include <queue>
#include <map>

using namespace std;

constexpr double inf = std::numeric_limits<double>::infinity();

template<class T>
void print_matrix(T **M, int N)
{
	cout << "\\\t";
	for (int v = 0; v < N; v++)
	{
		cout << v+1 << '\t';
	}
	cout << '\n';
	for (int u = 0; u < N; u++)
	{
		cout << u+1 << '\t';
		for (int v = 0; v < N; v++)
		{
			cout << M[u][v] << '\t';
		}
		cout << '\n';
	}
}


int floyd_warshall(double **ShortestPath, int **Previous_on_SP, int N)
{
	//инициализация матрицы предыдущих вершин на кратчайших путях
	for (int u = 0; u < N; u++)
	{
		for (int v = 0; v < N; v++)
		{
			Previous_on_SP[u][v] = u;
		}
	}

	//Нахождение кратчайших путей
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

		//проверка на наличие отрицательного цикла
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


list<int> find_negative_cycle(double **Graph, int N)
{
	//выделение памяти для матрицы кратчайших путей и матрицы предыдущих вершин на кратчайших путях
	double **ShortestPath = new double*[N];
	for (int i = 0; i < N; i++)
	{
		ShortestPath[i] = new double[N];
	}

	int **Previous_on_SP = new int*[N];
	for (int i = 0; i < N; i++)
	{
		Previous_on_SP[i] = new int[N];
	}

	//инициализация матрицы кратчайших путей
	for (int u = 0; u < N; u++)
	{
		for (int v = 0; v < N; v++)
		{
			ShortestPath[u][v] = Graph[u][v];
		}
	}

	int v_in_negative_cycle = floyd_warshall(ShortestPath, Previous_on_SP, N);
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

	//освобождение памяти
	for (int i = 0; i < N; i++)
	{
		delete[] ShortestPath[i];
	}
	delete[] ShortestPath;

	for (int i = 0; i < N; i++)
	{
		delete[] Previous_on_SP[i];
	}
	delete[] Previous_on_SP;

	return negative_cycle;
}


forward_list<int> bread_first_search_path(double **Graph, int N, int from, int to)
{
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
			if(Graph[u][v] != inf && Graph[u][v] != 0 && used[v] == false)
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


void build_increment(double **Capacity, double **Flow, double **incrementCp, int N)
{
	for (int u = 0; u < N; u++)
	{
		for (int v = 0; v < N; v++)
		{
			if (Flow[u][v] < Capacity[u][v])
			{
				incrementCp[u][v] = Capacity[u][v] - Flow[u][v];
			}
			else if (Flow[v][u] > 0)
			{
				incrementCp[u][v] = Flow[v][u];
			}
			else
			{
				incrementCp[u][v] = 0;
			}
		}
	}
}


void find_max_flow(double **Capacity, double **Flow, int N)
{
	double **incrementCp = new double*[N];
	for (int i = 0; i < N; i++)
	{
		incrementCp[i] = new double[N];
	}

	for (int u = 0; u < N; u++)
	{
		for (int v = 0; v < N; v++)
		{
			Flow[u][v] = 0;
		}
	}

	while (true)
	{
		build_increment(Capacity, Flow, incrementCp, N);
		forward_list<int> path = bread_first_search_path(incrementCp, N, 0, N-1);
		if (path.empty()) break;

		double delta = inf;
		int path_start = path.front();
		path.pop_front();
		int u = path_start;
		for (const auto &v : path)
		{
			if(incrementCp[u][v] < delta)
			{
				delta = incrementCp[u][v];
			}
			u = v;
		}

		u = path_start;
		for (const auto &v : path)
		{
			if(Capacity[u][v] != 0)
			{
				Flow[u][v] += delta;
			}
			else
			{
				Flow[v][u] -= delta;
			}
			u = v;
		}
	}

	//Освобождение памяти
	for (int i = 0; i < N; i++)
	{
		delete[] incrementCp[i];
	}
	delete[] incrementCp;
}


void find_flow(double **Capacity, double **Flow, int N, double flow_value)
{
	find_max_flow(Capacity, Flow, N);
	double max_flow_value = 0;
	for (int v = 0; v < N; v++)
	{
		max_flow_value += Flow[0][v];
	}

	double over_flow_value = max_flow_value - flow_value;
	while(over_flow_value > 0)
	{
		forward_list<int> path = bread_first_search_path(Flow, N, 0, N-1);
		int path_start = path.front();
		path.pop_front();
		double delta = over_flow_value;
		int u = path_start;
		for (const auto &v : path)
		{
			if (Flow[u][v] < delta)
			{
				delta = Flow[u][v];
			}
			u = v;
		}

		u = path_start;
		for (const auto &v : path)
		{
			Flow[u][v] -= delta;
			u = v;
		}

		over_flow_value -= delta;
	}
}


void find_min_cost_flow(double **Capacity, double **Cost, double **Flow, int N, double flow_value) //алгоритм поиска потока минимальной стоимости
{
	find_flow(Capacity, Flow, N, flow_value);

	double **incrementCp = new double*[N];
	for (int i = 0; i < N; i++)
	{
		incrementCp[i] = new double[N];
	}

	double **incrementCost = new double*[N];
	for (int i = 0; i < N; i++)
	{
		incrementCost[i] = new double[N];
	}

	while(true)
	{
		build_increment(Capacity, Flow, incrementCp, N);

		//определение стоимостей дуг инкрементального графа
		for (int u = 0; u < N; u++)
		{
			for (int v = 0; v < N; v++)
			{
				if (incrementCp[u][v] == 0) //если отсутствует дуга в инкрементальном графе
				{
					incrementCost[u][v] = (u==v ? 0 : inf);
				}
				else if (Capacity[u][v] == 0)
				{
					incrementCost[u][v] = -Cost[v][u];
				}
				else
				{
					incrementCost[u][v] = Cost[u][v];
				}
			}
		}

		//Нахождение отрицательного цикла
		list<int> negative_cycle = list<int>(find_negative_cycle(incrementCost, N));

		if (negative_cycle.empty()) break;

		//Перераспределение потоков
		double delta = inf;
		int u = negative_cycle.back();
		for (const auto &v : negative_cycle)
		{
			if (incrementCp[u][v] < delta) 
			{
				delta = incrementCp[u][v];
			}
			u = v;
		}

		u = negative_cycle.back();
		for (const auto &v : negative_cycle)
		{
			if(incrementCost[u][v] < 0) Flow[v][u] -= delta;
			else if (incrementCost[u][v] > 0) Flow[u][v] += delta;
			u = v;
		}
	}

	//Освобождение памяти
	for (int i = 0; i < N; i++)
	{
		delete[] incrementCp[i];
	}
	delete[] incrementCp;

	for (int i = 0; i < N; i++)
	{
		delete[] incrementCost[i];
	}
	delete[] incrementCost;
}


//reads graph from file and represents it as map that associates edges with weights
//the file must be in csv format
//columns: the first vertex of the edge, the second vertex of the edge, weight
template<class T> //T - type of graph weights
map<pair<int,int>, T> read_graph(string filename)
{
	ifstream file(filename);
	char delimiter = ',';
	map<pair<int,int>, T> graph;
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
		stream_line >> graph[{stoi(u), stoi(v)}];
	}
	file.close();
	return graph;
}


int main()
{
	const int N = 7;

	double Capacity[N][N] = //матрица пропускных способностей
	{ // 1, 2,  3,  4,  5,  6,  7
		{0, 16, 0,  11, 0,  13, 0}, //1
		{0, 0,  17, 18, 16, 0,  0}, //2
		{0, 0,  0,  0,  0,  0,  22},//3
		{0, 0,  12, 0,  0,  19, 0}, //4
		{0, 0,  0,  0,  0,  0,  16},//5
		{0, 0,  0,  0,  10, 0,  5}, //6
		{0, 0,  0,  0,  0,  0,  0}  //7
	};

	double Cost[N][N] = //матрица стоимостей
	{ // 1,   2,   3,   4,   5,   6,   7
		{0,   7,   inf, 13,  inf, 28,  inf},//1
		{inf, 0,   25,  4,   10,  inf, inf},//2
		{inf, inf, 0,   inf, inf, inf, 5},  //3
		{inf, inf, 6,   0,   inf, 5,   inf},//4
		{inf, inf, inf, inf, 0,   inf, 12}, //5
		{inf, inf, inf, inf, 3,   0,   7},  //6
		{inf, inf, inf, inf, inf, inf, 0}   //7
	};

	double Flow[N][N]; //матрица потоков

	double* pCapacity[N];
	double* pCost[N];
	double* pFlow[N];
	for (int i = 0; i < N; i++)
	{
		pCapacity[i] = &Capacity[0][0] + i*N;
		pCost[i] = &Cost[0][0] + i*N;
		pFlow[i] = &Flow[0][0] + i*N;
	}

	find_min_cost_flow(pCapacity, pCost, pFlow, N, 20);
	print_matrix(pFlow, N);
	return 0;
}