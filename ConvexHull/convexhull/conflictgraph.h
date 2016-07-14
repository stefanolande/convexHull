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
    std::map<Dcel::Vertex*, std::set<Dcel::Face*>*> pointMap;
    std::map<Dcel::Face*, std::set<Dcel::Vertex*>*> faceMap;
public:
    ConflictGraph(DrawableDcel* dcel, std::vector<Dcel::Vertex*> &vertexVec);
    std::set<Dcel::Face*>* getVisibleFaces(Dcel::Vertex* vertex);
    std::set<Dcel::Vertex *>* getVisibleVertices(Dcel::Face* face);
private:
    void addToFaceMap(Dcel::Face* face, Dcel::Vertex* vertexToAdd);
    void addToPointMap(Dcel::Vertex* vertex, Dcel::Face* faceToAdd);
};

#endif // CONFLICTGRAPH_H
