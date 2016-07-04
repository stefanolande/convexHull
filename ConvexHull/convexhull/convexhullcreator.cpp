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

    //add the 4 point
    this->dcel->addVertex(*this->vertexVec[0]);
    this->dcel->addVertex(*this->vertexVec[1]);
    this->dcel->addVertex(*this->vertexVec[2]);
    this->dcel->addVertex(*this->vertexVec[3]);

    //add the 12 half edges and set the twins
    for(int i=0; i<4; i++){
        for(int j=i+1; j<4; j++){
            Dcel::HalfEdge* heOut = this->dcel->addHalfEdge();
            heOut->setFromVertex(this->vertexVec[i]);
            heOut->setToVertex(this->vertexVec[j]);

            Dcel::HalfEdge* heIn = this->dcel->addHalfEdge();
            heIn->setFromVertex(this->vertexVec[j]);
            heIn->setToVertex(this->vertexVec[i]);

            heOut->setTwin(heIn);
            heIn->setTwin(heOut);
        }
    }


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
