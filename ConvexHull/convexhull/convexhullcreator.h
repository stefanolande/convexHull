#ifndef CONVEX_HULL_CREATOR_H
#define CONVEX_HULL_CREATOR_H

#include <vector>
#include "lib/dcel/drawable_dcel.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include "conflictgraph.h"

class ConvexHullCreator {
    std::vector<Pointd> vertexVec;
    DrawableDcel* dcel;
public:
    ConvexHullCreator(DrawableDcel* dcel);
    void calculate();
private:
    void getVertices();
    void findValidPermutation();
    void createTetrahedron();
    Dcel::Face *addFace(Dcel::Vertex* vertex, Dcel::HalfEdge* he);
    void setTwins(std::vector<Dcel::Face *> &faceList);
    void checkSanity();
    void removeHalfEdgeList(std::list<Dcel::HalfEdge*> &heList);
    void adjustTwin1(Dcel::HalfEdge *he, std::list<Dcel::Face *> &faceList);
    Dcel::Face *addFaceForTetrahedron(Dcel::Vertex *otherVertex, Dcel::HalfEdge *existingHe);
    void removeVisibleFaces(std::set<Dcel::Face *> &faceList);
    std::list<Dcel::HalfEdge*> getHorizon(std::set<Dcel::Face*>* visibleFaces);

};

#endif
