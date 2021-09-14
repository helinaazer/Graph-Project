#include <iostream>
#include <string>

#include "edge.h"

using namespace std;

// constructor with from, to, and weight
Edge::Edge(Vertex *from, Vertex *to, int weight)
    : _from(from), _to(to), _weight(weight) {}

// return the weight of the edge
int Edge::getEdgeWeight() const { return _weight; }

// return the vertex the edge is connected to
Vertex *Edge::getTargetVertex() const { return _to; }
