#ifndef CONFLICTGRAPH_H
#define CONFLICTGRAPH_H

#include <eigen3/Eigen/Dense>
#include "lib/dcel/drawable_dcel.h"
#include <map>
#include <vector>
#include <unordered_set>

class ConflictGraph
{
    DrawableDcel* dcel;
    std::list<Pointd> pointList;
    //std::map<Dcel::Vertex*, std::set<Dcel::Face*>*> pointMap;
    //std::map<Dcel::Face*, std::set<Dcel::Vertex*>*> faceMap;

    std::set<std::pair<Pointd, Dcel::Face*>> conflict;
public:
    ConflictGraph(DrawableDcel* dcel, const std::vector<Pointd> &pointList);
    std::unordered_set<Dcel::Face *> *getVisibleFaces(Pointd &vertex);
    std::set<Pointd> *getVisibleVertices(Dcel::Face *face);
    bool checkVisibility(Dcel::Face* face, const Pointd &vertex);
    void updateConflictGraph(Dcel::Face* face, const std::set<Pointd*> &candidateVertices);
    void deleteFaces(std::unordered_set<Dcel::Face*>* faces);
    void deletePoint(Pointd &vertex);
    void updateNaive(Dcel::Face *face);
private:
    //void addToFaceMap(Dcel::Face* face, const Pointd &vertexToAdd);
    //void addToPointMap(const Pointd &vertex, Dcel::Face* faceToAdd);
};

#endif // CONFLICTGRAPH_H
