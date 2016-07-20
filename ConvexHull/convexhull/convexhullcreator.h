#ifndef CONVEX_HULL_CREATOR_H
#define CONVEX_HULL_CREATOR_H

#include <vector>
#include "lib/dcel/drawable_dcel.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <unordered_set>
#include <unordered_map>
#include <random>
#include "conflictgraph.h"

class ConvexHullCreator {
public:
    ConvexHullCreator(DrawableDcel* dcel);
    void calculate();
private:
    std::vector<Pointd> pointVec;
    DrawableDcel *dcel;
    ConflictGraph *conflictGraph;

    void getVertices();
    void findValidPermutation();
    void createTetrahedron();
    Dcel::Face *addFace(Dcel::Vertex* vertex, Dcel::HalfEdge* he);
    void setTwins(std::vector<Dcel::Face *> &faceList);
    Dcel::Face *addFaceForTetrahedron(Dcel::Vertex *otherVertex, Dcel::HalfEdge *existingHe);
    void removeVisibleFaces(std::unordered_set<Dcel::Face *> &faceList);
    std::list<Dcel::HalfEdge*> getHorizon(std::unordered_set<Dcel::Face *> *visibleFaces);   
    std::unordered_map<Dcel::HalfEdge *, std::unordered_set<Pointd> *> getCandidateVertexMap(std::list<Dcel::HalfEdge *> horizon);
};

#endif
