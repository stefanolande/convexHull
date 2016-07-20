#ifndef CONFLICTGRAPH_H
#define CONFLICTGRAPH_H

#include <eigen3/Eigen/Dense>
#include "lib/dcel/drawable_dcel.h"
#include <map>
#include <vector>
#include <unordered_set>

namespace std {
template<>
class hash<Pointd> {
public:
    size_t operator()(const Pointd& k) const
    {
        using std::size_t;
        using std::hash;

        return ((hash<double>()(k.x())
                 ^ (hash<double>()(k.y()) << 1)) >> 1)
                ^ (hash<double>()(k.z()) << 1);
    }
};

template<>
class hash<pair<Pointd, Dcel::Face*>> {
public:
    size_t operator()(const pair<Pointd, Dcel::Face*>& k) const
    {
        using std::size_t;
        using std::hash;

        return (hash<Pointd>()(k.first)
                 ^ (hash<Dcel::Face*>()(k.second) << 1));
    }
};
}

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
