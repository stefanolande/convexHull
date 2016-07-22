#ifndef CONFLICTGRAPH_H
#define CONFLICTGRAPH_H

#include <eigen3/Eigen/Dense>
#include "lib/dcel/drawable_dcel.h"
#include "hashes.h"
#include <map>
#include <vector>
#include <unordered_set>
#include <unordered_map>

class ConflictGraph2
{    
public:
    ConflictGraph2(DrawableDcel* dcel, const std::vector<Pointd> &pointList);
    std::unordered_set<Dcel::Face *> *getVisibleFaces(Pointd &vertex);
    std::unordered_set<Pointd> *getVisibleVertices(Dcel::Face *face);
    void updateConflictGraph(Dcel::Face* face, std::unordered_set<Pointd> *candidateVertices);
    void deleteFaces(std::unordered_set<Dcel::Face *> &faces);
    void deletePoint(Pointd &vertex);
    void updateNaive(Dcel::Face *face);
private:
    DrawableDcel* dcel;
    std::list<Pointd> pointList;

    std::unordered_map<Pointd, std::unordered_set<Dcel::Face*>*> Fconflict;
    std::unordered_map<Dcel::Face*, std::unordered_set<Pointd>*> Pconflict;

    bool checkVisibility(Dcel::Face* face, const Pointd &vertex);
    void insertInFconflict(Pointd point, Dcel::Face *face);
    void insertInPconflict(Pointd point, Dcel::Face *face);
};

#endif // CONFLICTGRAPH_H
