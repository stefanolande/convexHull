#ifndef CONVEX_HULL_CREATOR_H
#define CONVEX_HULL_CREATOR_H

#include <vector>
#include "lib/dcel/drawable_dcel.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

class ConvexHullCreator {
    std::vector<Dcel::Vertex*> vertexVec;
    DrawableDcel* dcel;
public:
    ConvexHullCreator(DrawableDcel* dcel);
    void calculate();
private:
    void getVertices();
    void createTetrahedron();
};

#endif
