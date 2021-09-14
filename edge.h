#ifndef EDGE_H
#define EDGE_H
 
#include "vertex.h"
 
class Vertex;
 
class Edge {
    friend class Vertex;
    friend class Graph;
 
public:
  // constructor with from, to, and weight
  Edge(Vertex *from, Vertex *to, int weight);
  // return the weight of the edge
  int getEdgeWeight() const;
  // return the vertex the edge is connected to
  Vertex *getTargetVertex() const;
    
private:
    //vertex pointer from
    Vertex* _from;
    //vertex pointer to
    Vertex* _to;
    //int weight
    int _weight;
    
    };
 
#endif //EDGE_H