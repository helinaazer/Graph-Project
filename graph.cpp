#include "graph.h"
#include "edge.h"
#include "vertex.h"
#include <algorithm>
#include <bits/stdc++.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

using namespace std;

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges) {
  dirOrNot = directionalEdges;
  numOfEdges = 0;
}

// destructor, deletes all verticies and edges
Graph::~Graph() {
  // for every vertex in _vertexMap, delete...
  for (auto &v : _vertexMap) {
    delete v.second;
  }
}

// @return total number of vertices
int Graph::verticesSize() const { return _vertexMap.size(); }

// @return total number of edges
int Graph::edgesSize() const { return numOfEdges; }

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const {
  auto searchItr = _vertexMap.find(label);
  if (searchItr == _vertexMap.end()) {
    return -1;
  }
  return searchItr->second->_edges.size();
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) {

  if (contains(label)) {
    return false;
  }

  auto *newVertex = new Vertex(label);
  _vertexMap[label] = newVertex;

  return true;
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  return _vertexMap.find(label) != _vertexMap.cend();
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const {
  auto searchItr = _vertexMap.find(label);
  if (searchItr == _vertexMap.end()) {
    return "";
  }
  string tempString = searchItr->second->generateConnectionString();
  return tempString;
}

// Add an edge between two vertices, create new vertices if necessary
// A vertex cannot connect to itself, cannot have P->P
// For digraphs (directed graphs), only one directed edge allowed, P->Q
// Undirected graphs must have P->Q and Q->P with same weight
// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight) {

  if (from == to) {
    return false;
  }
  if (!contains(from)) {
    add(from);
  }
  if (!contains(to)) {
    add(to);
  }
  for (auto &edge : _vertexMap[from]->_edges) {
    if (edge->_to->getLabel() == to) {
      return false;
    }
  }

  Edge *newEdge = new Edge(_vertexMap[from], _vertexMap[to], weight);

  _vertexMap[from]->_edges.emplace_back(newEdge);
  sort(_vertexMap[from]->_edges.begin(), _vertexMap[from]->_edges.end(),
       compareEdges);

  if (!dirOrNot) {
    Edge *newEdge1 = new Edge(_vertexMap[to], _vertexMap[from], weight);
    _vertexMap[to]->_edges.emplace_back(newEdge1);
    sort(_vertexMap[to]->_edges.begin(), _vertexMap[to]->_edges.end(),
         compareEdges);
  }
  numOfEdges++;
  return true;
}

// Remove edge from graph
// @return true if edge successfully deleted
bool Graph::disconnect(const string &from, const string &to) {

  Vertex *toVertex = _vertexMap[to];

  for (size_t i = 0; i < _vertexMap[from]->_edges.size(); i++) {
    if (_vertexMap[from]->_edges[i]->_to == toVertex) {
      delete _vertexMap[from]->_edges[i];
      _vertexMap[from]->_edges.erase(_vertexMap[from]->_edges.begin() + i);
      numOfEdges--;
      return true;
    }
  }

  return false;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  set<string> visited;
  dfsHelper(startLabel, visited, visit);
}

// depth-first search helper function, starts from current label
void Graph::dfsHelper(const string &currentLabel, set<string> &visited,
                      void visit(const string &label)) {
  if (!contains(currentLabel)) {
    return;
  }
  if (visited.find(currentLabel) != visited.end()) {
    return;
  }
  visited.insert(currentLabel);
  visit(currentLabel);
  auto vertex = _vertexMap[currentLabel];
  for (auto edge : vertex->_edges) {
    dfsHelper(edge->getTargetVertex()->getLabel(), visited, visit);
  }
}

// breadth-first traversal starting from startLabel
// call the function visit on each vertex label */
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    return;
  }

  set<string> visited;
  queue<string> processingQueue;
  processingQueue.push(startLabel);

  string currentLabel;
  while (!processingQueue.empty()) {
    currentLabel = processingQueue.front();
    processingQueue.pop();

    if (!contains(currentLabel)) {
      continue;
    }

    if (visited.find(currentLabel) != visited.end()) {
      continue;
    }

    visited.insert(currentLabel);
    visit(currentLabel);
    auto vertex = _vertexMap[currentLabel];
    for (auto edge : vertex->_edges) {
      processingQueue.push(edge->getTargetVertex()->getLabel());
    }
  }
}

// dijkstra's algorithm to find shortest distance to all other vertices
// and the path to all other vertices
// Path cost is recorded in the map passed in, e.g. weight["F"] = 10
// How to get to the vertex is recorded previous["F"] = "C"
// @return a pair made up of two map objects, Weights and Previous
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  map<string, int> weights;
  map<string, string> previous;
  set<string> visited;

  for (auto pair : _vertexMap) {
    pair.second->unvisit();
    if (pair.second->getLabel() == startLabel) {
      continue;
    }
    weights[pair.second->getLabel()] = INT_MAX;
  }

  dijkstraHelper(startLabel, 0, visited, weights, previous);

  for (auto &pair : _vertexMap) {

    if (weights.find(pair.second->getLabel()) == weights.end()) {
      continue;
    }
    if (weights[pair.second->getLabel()] == INT_MAX) {
      weights.erase(pair.second->getLabel());
    }
  }
  return make_pair(weights, previous);
}

// helper function for dijkstra's algorithm
void Graph::dijkstraHelper(const string &currentLabel, int currentWeight,
                           set<string> &visited, map<string, int> &weights,
                           map<string, string> &previous) const {
  auto searchItr = _vertexMap.find(currentLabel);
  if (searchItr == _vertexMap.end()) {
    return;
  }
  if (searchItr->second->isVisited()) {
    return;
  }
  searchItr->second->visit();

  auto vertex = searchItr->second;
  for (auto &edge : vertex->_edges) {
    auto targetVertex = edge->getTargetVertex();
    if (targetVertex->isVisited()) {
      continue;
    }

    int newWeight = currentWeight + edge->getEdgeWeight();
    if (newWeight < weights[targetVertex->getLabel()]) {
      weights[targetVertex->getLabel()] = newWeight;
      previous[targetVertex->getLabel()] = currentLabel;
    }
  }
  int minWeight = INT_MAX;
  string minLabel;
  for (auto &pair : _vertexMap) {

    if (pair.second->isVisited()) {
      continue;
    }

    if (weights.find(pair.second->getLabel()) != weights.cend()) {

      if (weights[pair.second->getLabel()] < minWeight) {
        minWeight = weights[pair.second->getLabel()];
        minLabel = pair.second->getLabel();
      }
    }
  }

  if (minWeight != INT_MAX) {
    dijkstraHelper(minLabel, weights[minLabel], visited, weights, previous);
  }
}

// minimum spanning tree using Prim's algorithm
// ONLY works for NONDIRECTED graphs
// ASSUMES the edge [P->Q] has the same weight as [Q->P]
// @return length of the minimum spanning tree or -1 if start vertex not
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  auto searchItr = _vertexMap.find(startLabel);
  if (searchItr == _vertexMap.end()) {
    return -1;
  }
  vector<Edge *> edges;
  set<string> visited;
  Vertex *currentVertex = searchItr->second;
  visited.insert(currentVertex->_label);
  auto targetEdges = currentVertex->_edges;
  edges.insert(edges.end(), targetEdges.begin(), targetEdges.end());

  int length = 0;

  while (visited.size() != verticesSize()) {
    size_t minIndex = 0;
    if (getMinimumWeightEdge(edges, minIndex, visited)) {

      length += edges[minIndex]->_weight;
      Vertex *prevVertex = edges[minIndex]->_from;
      Vertex *nextVertex = edges[minIndex]->_to;
      visited.insert(nextVertex->_label);

      visit(prevVertex->_label, nextVertex->_label, edges[minIndex]->_weight);

      edges.erase(edges.begin() + minIndex);
      for (auto newEdge : nextVertex->_edges) {
        edges.emplace_back(newEdge);
      }
    } else {
      break;
    }
  }
  return length;
}

// function that gets the minimum edge weight. Return false if no edge found
bool Graph::getMinimumWeightEdge(vector<Edge *> &edges, size_t &minIndex,
                                 set<string> &visited) const {
  int minimum = INT_MAX;
  bool found = false;
  for (size_t i = 0; i < edges.size(); ++i) {

    if (visited.find(edges[i]->getTargetVertex()->_label) != visited.end()) {
      continue;
    }

    found = true;

    if (minimum > edges[i]->getEdgeWeight()) {
      minimum = edges[i]->getEdgeWeight();
      minIndex = i;
    }
  }
  return found;
}

// minimum spanning tree using Kruskal's algorithm
// ONLY works for NONDIRECTED graphs
// ASSUMES the edge [P->Q] has the same weight as [Q->P]
// @return length of the minimum spanning tree or -1 if start vertex not
int Graph::mstKruskal(const string & /*startLabel*/,
                      void /*visit*/ (const string &from, const string &to,
                                      int weight)) const {
  return -1;
}

// read a text file and create the graph
bool Graph::readFile(const string &filename) {
  ifstream myfile(filename);
  if (!myfile.is_open()) {
    cerr << "Failed to open " << filename << endl;
    return false;
  }
  int edges = 0;
  int weight = 0;
  string fromVertex;
  string toVertex;
  myfile >> edges;
  for (int i = 0; i < edges; ++i) {
    myfile >> fromVertex >> toVertex >> weight;
    connect(fromVertex, toVertex, weight);
  }
  myfile.close();
  return true;
}

// function to compare edges. Used to print out the edges in alphabetical order
bool Graph::compareEdges(Edge *first, Edge *second) {
  return (first->_to->_label < second->_to->_label);
}
