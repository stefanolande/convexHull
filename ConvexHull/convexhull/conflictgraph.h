#ifndef CONFLICTGRAPH_H
#define CONFLICTGRAPH_H

#include <eigen3/Eigen/Dense>
#include "lib/dcel/drawable_dcel.h"
#include <map>
#include <vector>

class ConflictGraph
{
    DrawableDcel* dcel;
    std::vector<Dcel::Vertex*> vertexVec;
    std::map<Dcel::Vertex*, std::list<Dcel::Face*>> pointMap;
    std::map<Dcel::Face*, std::list<Dcel::Vertex*>> faceMap;
public:
    ConflictGraph(DrawableDcel* dcel, std::vector<Dcel::Vertex*> &vertexVec);
private:
    void addToFaceMap(Dcel::Face* face, Dcel::Vertex* vertexToAdd);
};

#endif // CONFLICTGRAPH_H
