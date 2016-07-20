#ifndef CONFLICTGRAPH_H
#define CONFLICTGRAPH_H

#include <eigen3/Eigen/Dense>
#include "lib/dcel/drawable_dcel.h"
#include <map>
#include <vector>
#include <unordered_set>

class ConflictGraph
{    
public:
    ConflictGraph(DrawableDcel* dcel, const std::vector<Pointd> &pointList);
    std::unordered_set<Dcel::Face *> *getVisibleFaces(Pointd &vertex);
    std::unordered_set<Pointd> *getVisibleVertices(Dcel::Face *face);
    void updateConflictGraph(Dcel::Face* face, std::unordered_set<Pointd> *candidateVertices);
    void deleteFaces(std::unordered_set<Dcel::Face*>* faces);
    void deletePoint(Pointd &vertex);
    void updateNaive(Dcel::Face *face);
private:
    DrawableDcel* dcel;
    std::list<Pointd> pointList;
    std::unordered_set<std::pair<Pointd, Dcel::Face*>> conflict;
    bool checkVisibility(Dcel::Face* face, const Pointd &vertex);

};

#endif // CONFLICTGRAPH_H
