#ifndef ARIEL_GRAPH_HPP
#define ARIEL_GRAPH_HPP

#include <vector>
#include <cstddef>
#include <iostream>

namespace ariel
{
    class Graph
    {
    private:
        std::vector<std::vector<int>> adjacencyMatrix;
        std::vector<bool> recStack;
        std::vector<bool> visited;
        bool isDirected;
        bool isWeighted;
        bool isCyclic;
        size_t numberOfEdges;
        size_t numberOfVertices;

    public:
        Graph()
        {
            isDirected = false;
            isWeighted = false;
            isCyclic = false;
            numberOfEdges = 0;
            numberOfVertices = 0;
        }
        std::vector<bool> getVisited() const;
        std::vector<std::vector<int>> getAdjacencyMatrix() const;
        bool isEdge(size_t v, size_t w) const;
        void printGraph() const;
        bool getIsCyclic();

        std::vector<int> getAd(size_t v);
        void loadGraph(const std::vector<std::vector<int>> &matrix);

        bool hasCycle();
        bool isCyclicUtil(size_t v, std::vector<bool> &visited, std::vector<bool> &recStack, std::vector<int> &path);
        void printCycle();
        size_t getVertices();
        size_t getNumEdges();
    };
}

#endif