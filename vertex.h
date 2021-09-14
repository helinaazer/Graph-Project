#ifndef VERTEX_H
#define VERTEX_H

#include "edge.h"
#include <string>
#include <vector>

using namespace std;

class Edge;

class Vertex {
  friend class Edge;
  friend class Graph;

public:
  // constructor
  explicit Vertex(const string &label);
  // copy constructor
  Vertex(const Vertex &v) = delete;
  // move not allowed
  Vertex(Vertex &&other) = delete;
  // assignment not allowed
  Vertex &operator=(const Vertex &other) = delete;
  // move assignment not allowed
  Vertex &operator=(Vertex &&other) = delete;
  // destructor
  ~Vertex();
  // return the label of the vertex
  string getLabel() const;
  // mark the vertex as visited
  void visit();
  // mark the vertex as unvisited
  void unvisit();
  // returns whether or not a vertex was visited. Returns true if visited and
  // falses if unvisited
  bool isVisited() const;
  // function used to print out the edge as a string. Function used in
  // graph.cpp
  string generateConnectionString() const;

private:
  // string label
  string _label;
  // vector of pointer edge
  vector<Edge *> _edges;
  // sets visited to false
  bool _visited{false};
};

#endif // VERTEX_H