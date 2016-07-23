#ifndef CONFLICTGRAPH_H
#define CONFLICTGRAPH_H

#include <eigen3/Eigen/Dense>
#include "lib/dcel/drawable_dcel.h"
#include "hashes.h"
#include <map>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include "hashlib.h"

class ConflictGraph2
{    
public:    
    ConflictGraph2(DrawableDcel* dcel, const std::vector<Pointd> &pointList);
    hashlib::pool<Dcel::Face *> *getVisibleFaces(Pointd &vertex);
    hashlib::pool<Pointd> *getVisibleVertices(Dcel::Face *face);
    void updateConflictGraph(Dcel::Face* face, hashlib::pool<Pointd> *candidateVertices);
    void deleteFaces(hashlib::pool<Dcel::Face *> *faces);
    void deletePoint(Pointd &vertex);
    void updateNaive(Dcel::Face *face);
private:
    DrawableDcel* dcel;
    std::list<Pointd> pointList;

    hashlib::dict<Pointd, hashlib::pool<Dcel::Face*>*> Fconflict;
    hashlib::dict<Dcel::Face*, hashlib::pool<Pointd>*> Pconflict;

    bool checkVisibility(Dcel::Face* face, const Pointd &vertex);
    void insertInFconflict(Pointd point, Dcel::Face *face);
    void insertInPconflict(Pointd point, Dcel::Face *face);
};

#endif // CONFLICTGRAPH_H
