#ifndef CONVEX_HULL_CREATOR_H
#define CONVEX_HULL_CREATOR_H

#include <vector>
#include "lib/dcel/drawable_dcel.h"
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <cstdlib>
#include <random>
#include "conflictgraph.h"
#include "GUI/mainwindow.h"

class ConvexHullCreator {
public:
    ConvexHullCreator(DrawableDcel* dcel);
    void calculate();
    void calculate(MainWindow *mainWindow);
private:
    std::vector<Pointd> pointVec;
    DrawableDcel *dcel;
    ConflictGraph *conflictGraph;

    void getVertices();
    void findValidPermutation();
    void createTetrahedron();
    Dcel::Face *addFace(Dcel::Vertex* vertex, Dcel::HalfEdge* he);
    void setTwins(std::vector<Dcel::Face *> &faceList);
    void addFaceForTetrahedron(Dcel::Vertex *otherVertex, Dcel::HalfEdge *existingHe);
    void removeVisibleFaces(hashlib::pool<Dcel::Face *> *faceList);
    std::list<Dcel::HalfEdge*> getHorizon(hashlib::pool<Dcel::Face *> *visibleFaces);
    hashlib::dict<Dcel::HalfEdge *, hashlib::pool<Pointd> *> getCandidateVerticesMap(std::list<Dcel::HalfEdge *> horizon);
};

#endif
