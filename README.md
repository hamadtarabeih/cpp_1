# cpp_1
﻿# Class Documentation

## `class ariel::Graph`

This class represents a graph data structure. It provides methods for loading a graph from an adjacency matrix, checking if the graph is cyclic, getting the number of vertices and edges, and more.

### Member Variables

- `adjacencyMatrix`: A 2D vector representing the adjacency matrix of the graph.
- `recStack`: A vector indicating whether each vertex is in the recursion stack or not.
- `visited`: A vector indicating whether each vertex has been visited or not.
- `isDirected`: A boolean indicating whether the graph is directed or not.
- `isWeighted`: A boolean indicating whether the graph is weighted or not.
- `isCyclic`: A boolean indicating whether the graph is cyclic or not.
- `numberOfEdges`: The number of edges in the graph.
- `numberOfVertices`: The number of vertices in the graph.

### Member Functions

- `Graph()`: Constructor that initializes the graph as undirected, unweighted, and acyclic, with 0 vertices and 0 edges.
- `std::vector<bool> getVisited() const`: Returns the visited vector.
- `std::vector<std::vector<int>> getAdjacencyMatrix() const`: Returns the adjacency matrix of the graph.
- `bool isEdge(size_t v, size_t w) const`: Checks if there is an edge between two given vertices.
- `void printGraph() const`: Prints the graph.
- `bool getIsCyclic()`: Returns whether the graph is cyclic or not.
- `std::vector<int> getAd(size_t v)`: Returns the adjacency list of a given vertex.
- `void loadGraph(const std::vector<std::vector<int>> &matrix)`: Loads a graph from a given adjacency matrix.
- `bool hasCycle()`: Checks if the graph has a cycle.
- `bool isCyclicUtil(size_t v, std::vector<bool> &visited, std::vector<bool> &recStack, std::vector<int> &path)`: A utility function used by `hasCycle()` to check if the graph has a cycle.
- `void printCycle()`: Prints a cycle in the graph if there is one.
- `size_t getVertices()`: Returns the number of vertices in the graph.
- `size_t getNumEdges()`: Returns the number of edges in the graph.

### Example

```cpp
ariel::Graph graph;
std::vector<std::vector<int>> matrix = {{0, 1, 0, 1}, {1, 0, 1, 0}, {0, 1, 0, 1}, {1, 0, 1, 0}};
graph.loadGraph(matrix);
graph.printGraph();
if (graph.getIsCyclic()) {
    graph.printCycle();
}
```

This will load a graph from the given adjacency matrix, print the graph, and if the graph is cyclic, print a cycle in the graph.


## `class ariel::Algorithms`

### Description

This class provides a set of static methods for performing various algorithms on a graph. The graph is represented by an object of the `ariel::Graph` class.

### Member Functions

#### `static int Algorithms::isConnected(ariel::Graph &graph)`

##### Description

This function checks if a given graph is connected.

##### Parameters

- `graph`: A reference to an object of the `ariel::Graph` class.

##### Return Value

Returns `1` if the graph is connected, and `0` otherwise.

#### `static std::string Algorithms::shortestPath(ariel::Graph &graph, int s, int e)`

##### Description

This function finds the shortest path between two vertices in a given graph.

##### Parameters

- `graph`: A reference to an object of the `ariel::Graph` class.
- `s`: The starting vertex.
- `e`: The ending vertex.

##### Return Value

Returns a string representation of the shortest path.

#### `static std::string Algorithms::isContainsCycle(ariel::Graph& graph)`

##### Description

This function checks if a given graph contains a cycle.

##### Parameters

- `graph`: A reference to an object of the `ariel::Graph` class.

##### Return Value

Returns `"true"` if the graph contains a cycle, and `"false"` otherwise.

#### `static void Algorithms::negativeCycle(ariel::Graph &graph)`

##### Description

This function checks if a given graph contains a negative weight cycle.

##### Parameters

- `graph`: A reference to an object of the `ariel::Graph` class.

##### Return Value

This function does not return a value. Instead, it prints a message to the standard output indicating whether the graph contains a negative weight cycle or not.

#### `static std::string Algorithms::isBipartite(ariel::Graph &graph)`

##### Description

This function checks if a given graph is bipartite.

##### Parameters

- `graph`: A reference to an object of the `ariel::Graph` class.

##### Return Value

Returns `"true"` if the graph is bipartite, and `"false"` otherwise.

#### `static void Algorithms::DFS(std::vector<bool>::size_type vertex, std::vector<int> &visited, std::vector<std::vector<int>> &adjacencyMatrix)`

##### Description

This function performs a Depth-First Search (DFS) from a given vertex in a graph represented by an adjacency matrix.

##### Parameters

- `vertex`: The vertex from which the DFS starts.
- `visited`: A reference to a vector that indicates whether each vertex has been visited or not.
- `adjacencyMatrix`: A reference to the adjacency matrix of the graph.

##### Return Value

This function does not return a value.

### Example

```cpp
ariel::Graph graph;
// add edges to the graph
std::cout << ariel::Algorithms::isConnected(graph) << std::endl;
std::cout << ariel::Algorithms::shortestPath(graph, 0, 3) << std::endl;
std::cout << ariel::Algorithms::isContainsCycle(graph) << std::endl;
ariel::Algorithms::negativeCycle(graph);
std::cout << ariel::Algorithms::isBipartite(graph) << std::endl;
std::vector<int> visited(graph.getVertices(), 0);
ariel::Algorithms::DFS(0, visited, graph.getAdjacencyMatrix());
```

This will check if the graph is connected, find the shortest path from vertex 0 to vertex 3, check if the graph contains a cycle, check if the graph contains a negative weight cycle, check if the graph is bipartite, and perform a DFS from vertex 0.

## isConnected

 `int ariel::Algorithms::isConnected(ariel::Graph &graph)`

This function checks if a given graph is connected or not. A graph is connected if there is a path from any vertex to any other vertex.

 `graph`: A reference to an object of the `ariel::Graph` class. This is the graph that the function will check for connectivity.

Returns an integer that indicates whether the graph is connected or not. If the graph is connected, the function returns `1`. If the graph is not connected, the function returns `0`.

### Algorithm

The function uses a Depth-First Search (DFS) based approach for checking connectivity. It starts by initializing a `visited` vector with `false` for all vertices, indicating that no vertex has been visited yet. It also gets the adjacency matrix of the graph.

The function then starts a DFS from the first vertex (vertex 0), marking visited vertices in the `visited` vector. After the DFS, it checks the `visited` vector. If there is any vertex that was not visited during the DFS (i.e., its corresponding value in the `visited` vector is `false`), it means the graph is not connected, so it returns `0`.

If all vertices were visited (i.e., all values in the `visited` vector are `true`), it means the graph is connected. In this case, the function returns `1`.

### Helper Function

- `void Algorithms::DFS(std::vector<bool>::size_type vertex, std::vector<int> &visited, std::vector<std::vector<int>> &adjacencyMatrix)`: This is a helper function that performs a DFS from a given vertex, marking visited vertices in the `visited` vector. It iterates over all neighbors of the given vertex, and for each unvisited neighbor that is connected to the given vertex, it recursively performs a DFS from it.

### Example

```cpp
ariel::Graph graph;
// add edges to the graph
int result = ariel::Algorithms::isConnected(graph);
std::cout << (result ? "The graph is connected." : "The graph is not connected.") << std::endl;
```

This will output "The graph is connected." if the graph is connected, or "The graph is not connected." if the graph is not connected.

## shortestPath

`std::string Algorithms::shortestPath(ariel::Graph &graph, int s, int e)`

This function finds the shortest path between two vertices in a given graph using the Breadth-First Search (BFS) algorithm.

`graph`: A reference to an object of the `ariel::Graph` class. This is the graph in which the function will find the shortest path.
 `s`: An integer that represents the start vertex.
 `e`: An integer that represents the end vertex.

Returns a string that represents the shortest path from the start vertex to the end vertex. If there's no path from the start vertex to the end vertex, the function returns "-1".

### Algorithm

The function starts by casting the start and end vertices to `size_t` type and initializing a queue `q`, a `distances` map to keep track of the distance from the start vertex to each vertex, and a `previous` map to keep track of the previous vertex on the shortest path.

It sets the distance from the start vertex to itself as 0 and adds the start vertex to the queue. It then enters a loop that continues until the queue is empty. In each iteration, it dequeues a vertex `current` from the queue.

If `current` is the end vertex, it constructs the shortest path string by traversing the `previous` map from the end vertex to the start vertex, and returns this path.

If `current` is not the end vertex, it iterates over its adjacent vertices. If an adjacent vertex `i` is not visited (i.e., it's not in the `distances` map) and there's an edge between `current` and `i` (i.e., `graph.getAdjacencyMatrix()[current][i] != 0`), it adds `i` to the queue, sets `distances[i]` as `distances[current] + 1`, and sets `previous[i]` as `current`.

If the queue becomes empty and it hasn't returned yet, it means there's no path from the start vertex to the end vertex. In this case, it returns "-1".

### Example

```cpp
ariel::Graph graph;
// add edges to the graph
std::string path = Algorithms::shortestPath(graph, 0, 3);
std::cout << "The shortest path from 0 to 3 is: " << path << std::endl;
```

This will output something like "The shortest path from 0 to 3 is: 0->1->3." if there's a path from 0 to 3, or "The shortest path from 0 to 3 is: -1." if there's no such path.

## negativeCycle
 `void Algorithms::negativeCycle(ariel::Graph &graph)`


This function checks if a given graph contains a negative weight cycle using the Bellman-Ford algorithm. A negative weight cycle is a cycle in a graph where the sum of the weights of the edges in the cycle is negative.
 `graph`: A reference to an object of the `ariel::Graph` class. This is the graph that the function will check for a negative weight cycle.


This function does not return a value. Instead, it prints a message to the standard output indicating whether the graph contains a negative weight cycle or not.

### Algorithm

The function starts by getting the number of vertices `V` in the graph and initializing a `dist` vector of size `V` with all elements set to `INT_MAX`, which represents infinity. It then sets the distance from the first vertex (vertex 0) to itself as 0.

It performs the relaxation process for `V - 1` times. In each iteration, it goes through all edges of the graph (represented by the adjacency matrix), and for each edge `(j, k)` with weight `graph.getAdjacencyMatrix()[j][k]`, if the distance to vertex `j` is not infinity and the distance to vertex `j` plus the weight of the edge is less than the current distance to vertex `k`, it updates the distance to vertex `k` with the new value.

After the relaxation process, it goes through all edges of the graph again. If it can still find an edge `(j, k)` that can be relaxed (i.e., the distance to vertex `j` is not infinity and the distance to vertex `j` plus the weight of the edge is less than the current distance to vertex `k`), it means the graph contains a negative weight cycle. In this case, it prints "Graph contains a negative weight cycle" and returns.

If it hasn't returned after checking all edges, it means the graph doesn't contain a negative weight cycle. In this case, it prints "No negative weight cycle found".

### Example

```cpp
ariel::Graph graph;
// add edges to the graph
Algorithms::negativeCycle(graph);
```

This will output "Graph contains a negative weight cycle" if the graph contains a negative weight cycle, or "No negative weight cycle found" if the graph doesn't contain a negative weight cycle.

## isBipartite


 `std::string Algorithms::isBipartite(ariel::Graph &graph)`

This function checks if a given graph is bipartite or not. A bipartite graph is a graph whose vertices can be divided into two disjoint and independent sets U and V such that every edge connects a vertex in U to one in V.

`graph`: A reference to an object of the `ariel::Graph` class. This is the graph that the function will check for bipartiteness.

Returns a string that indicates whether the graph is bipartite or not. If the graph is bipartite, the string will also include the two sets of vertices. If the graph is not bipartite, the function returns "0".

### Algorithm

The function uses a Breadth-First Search (BFS) based approach for checking bipartiteness. It starts by initializing a color array `colorArr` with `-1` for all vertices, indicating that no vertex has been assigned a set yet. It also initializes two vectors `setA` and `setB` to store the vertices of the two sets.

The function then iterates over all vertices. For each uncolored vertex `i`, it colors it with `1`, adds it to `setA`, and starts a BFS from it. In the BFS, for each uncolored neighbor `v` of the current vertex `u`, it colors `v` with `1 - colorArr[u]` (i.e., if `u` is colored with `1`, `v` is colored with `0`, and vice versa), adds `v` to the corresponding set, and enqueues `v`. If it finds a neighbor `v` that is already colored and has the same color as `u`, it means the graph is not bipartite, so it returns "0".

After checking all vertices, if the function hasn't returned yet, it means the graph is bipartite. It then constructs the return string that includes the two sets of vertices and returns this string.

### Example

```cpp
ariel::Graph graph;
// add edges to the graph
std::string result = Algorithms::isBipartite(graph);
std::cout << result << std::endl;
```

This will output something like "The graph is bipartite: A={0, 2}, B={1, 3}." if the graph is bipartite, or "0" if the graph is not bipartite.



