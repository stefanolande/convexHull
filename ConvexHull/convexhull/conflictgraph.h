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
    std::list<Dcel::Face*> getVisibleFaces(Dcel::Vertex* vertex);
    std::list<Dcel::Vertex *> getVisibleVertices(Dcel::Face* face);
private:
    void addToFaceMap(Dcel::Face* face, Dcel::Vertex* vertexToAdd);
    void addToPointMap(Dcel::Vertex* vertex, Dcel::Face* faceToAdd);
};

#endif // CONFLICTGRAPH_H
