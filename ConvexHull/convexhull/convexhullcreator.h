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
    std::vector<Dcel::Vertex*> vertexVec;
    DrawableDcel* dcel;
public:
    ConvexHullCreator(DrawableDcel* dcel);
    void calculate();
private:
    void getVertices();
    void findValidPermutation();
    void createTetrahedron();
    void addFace(Dcel::Vertex* vertex, Dcel::HalfEdge* he);
    void adjustTwin(Dcel::HalfEdge* he);
};

#endif
