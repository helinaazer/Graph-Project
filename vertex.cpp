#include "vertex.h"

#include "edge.h"
#include <functional>
#include <iostream>
#include <map>
#include <string>

using namespace std;

// constructor
Vertex::Vertex(const string &label) {
  _visited = false;
  _label = label;
}

// destructor
Vertex::~Vertex() {
  // for each edge, delete
  for (Edge *e : _edges) {
    delete e;
  }
}

// return the label of the vertex
string Vertex::getLabel() const { return _label; }

// mark vertex as visited
void Vertex::visit() { _visited = true; }

// mark vertex as unvisitied
void Vertex::unvisit() { _visited = false; }

// returns whether or not a vertex was visited. Returns true if visited and
// falses if unvisited
bool Vertex::isVisited() const { return _visited; }

// function used to print out the edge as a string. Function used in graph.cpp
string Vertex::generateConnectionString() const {
  string s;
  string comma;
  for (Edge *e : _edges) {
    string edgeString =
        comma + e->_to->_label + "(" + to_string(e->_weight) + ")";
    s += edgeString;
    comma = ",";
  }
  return s;
}