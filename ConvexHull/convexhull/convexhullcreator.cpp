#include "convexhullcreator.h"

ConvexHullCreator::ConvexHullCreator(DrawableDcel* dcel){
    this->dcel = dcel;
    this->vertexVec = std::vector<Dcel::Vertex*>(dcel->getNumberVertices());
}


void ConvexHullCreator::calculate(){

    //get the vertices list from the dcel
    getVertices();

    //clear the initial dcel
    dcel->reset();

    //get a random permutation that starts with 4 non coplanar vertices
    bool coplanar = true;

    do{
        //calculate a random permutation of the vertices vector
        std::random_shuffle(this->vertexVec.begin(), this->vertexVec.end());

        //check if the first point are coplanar
        Eigen::MatrixXd matrix(4,4);
        for(int i=0; i<4; i++){
            matrix(i, 0) = this->vertexVec[i]->getCoordinate().x();
            matrix(i, 1) = this->vertexVec[i]->getCoordinate().y();
            matrix(i, 2) = this->vertexVec[i]->getCoordinate().z();
            matrix(i, 3) = 1;
        }

        double det = matrix.determinant();

        std::cout << det << std::endl;

        //check if the determinant is 0 +- epsilon
        coplanar = det > -std::numeric_limits<double>::epsilon() && det < std::numeric_limits<double>::epsilon();

    } while(coplanar);

    //create the inital tetrahedron
    createTetrahedron();
}

/**
 * @brief ConvexHullCreator::createTetrahedron
 */
void ConvexHullCreator::createTetrahedron(){
    //add the 4 point
    Dcel::Vertex* a = this->dcel->addVertex(*this->vertexVec[0]);
    Dcel::Vertex* b = this->dcel->addVertex(*this->vertexVec[1]);
    Dcel::Vertex* c = this->dcel->addVertex(*this->vertexVec[2]);
    Dcel::Vertex* d = this->dcel->addVertex(*this->vertexVec[3]);

    Dcel::HalfEdge* he1In = this->dcel->addHalfEdge();
    Dcel::HalfEdge* he2In = this->dcel->addHalfEdge();
    Dcel::HalfEdge* he3In = this->dcel->addHalfEdge();

    he1In->setFromVertex(b);
    he1In->setToVertex(a);
    he1In->setNext(he3In);
    he1In->setPrev(he2In);

    he2In->setFromVertex(c);
    he2In->setToVertex(b);
    he2In->setNext(he1In);
    he2In->setPrev(he3In);

    he3In->setFromVertex(a);
    he3In->setToVertex(c);
    he3In->setNext(he2In);
    he3In->setPrev(he1In);

    Dcel::Face* face1 = this->dcel->addFace();
    face1->setOuterHalfEdge(he1In);
    he1In->setFace(face1);
    he2In->setFace(face1);
    he3In->setFace(face1);

    std::cout << "#HE " << dcel->getNumberHalfEdges() << std::endl;
}


void ConvexHullCreator::getVertices(){
    Dcel::VertexIterator vit;
    int i=0;
    for(vit = dcel->vertexBegin(); vit != dcel->vertexEnd(); ++vit){
        this->vertexVec[i] = *vit;
        i++;
    }
}
