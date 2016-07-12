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
    
    findValidPermutation();
    
    //create the inital tetrahedron
    createTetrahedron();

    ConflictGraph conflictGraph(dcel, vertexVec);
}

/**
 * @brief ConvexHullCreator::findValidPermutation
 * Does a random shuffle on the input vertices vector. The first four must not be coplanar to form a tetrahedron.
 */
void ConvexHullCreator::findValidPermutation(){
    //get a random permutation that starts with 4 non coplanar vertices
    bool coplanar = true;
    
    std::srand(std::time(0));
    
    do{
        //calculate a random permutation of the vertices vector
        std::random_shuffle(this->vertexVec.begin(), this->vertexVec.end());
        
        //check if the first point are coplanar
        Eigen::Matrix4d matrix;
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

        //TODO: swap two elements instead of reshuffling the whole vector if coplanar
        
    } while(coplanar);
}

/**
 * @brief ConvexHullCreator::createTetrahedron
 * This method create the tetrahedon needed to start the incremental step of the convex hull algortihm
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
    
    addFace(d, he1In);
    addFace(d, he2In);
    addFace(d, he3In);
    
    std::cout << "#HE " << dcel->getNumberHalfEdges() << std::endl;
}

/**
 * @brief ConvexHullCreator::addFace
 * Add a face connected to an existing vertex and an existing vertex. Used to attach new faces.
 * @param otherVertex
 * @param existingHe
 */
void ConvexHullCreator::addFace(Dcel::Vertex* otherVertex, Dcel::HalfEdge* existingHe){
    Dcel::HalfEdge* he1 = this->dcel->addHalfEdge();
    Dcel::HalfEdge* he2 = this->dcel->addHalfEdge();
    Dcel::HalfEdge* he3 = this->dcel->addHalfEdge();
    
    Dcel::Vertex* startVertex = existingHe->getFromVertex();
    Dcel::Vertex* endVertex = existingHe->getToVertex();
    
    he1->setFromVertex(endVertex);
    he1->setToVertex(startVertex);
    he1->setNext(he2);
    he1->setPrev(he3);
    he1->setTwin(existingHe);
    
    he2->setFromVertex(startVertex);
    he2->setToVertex(otherVertex);
    he2->setNext(he3);
    he2->setPrev(he1);
    adjustTwin(he2);
    
    he3->setFromVertex(otherVertex);
    he3->setToVertex(endVertex);
    he3->setNext(he1);
    he3->setPrev(he2);
    adjustTwin(he3);
    
    Dcel::Face* face = this->dcel->addFace();
    face->setOuterHalfEdge(he1);
    he1->setFace(face);
    he2->setFace(face);
    he3->setFace(face);
}

/**
 * @brief ConvexHullCreator::adjustTwin
 * Given the input half hedge, the method finds its twin set it
 * @param he
 */
void ConvexHullCreator::adjustTwin(Dcel::HalfEdge* he){
    Dcel::Vertex* startVertex = he->getFromVertex();
    Dcel::Vertex* endVertex = he->getToVertex();
    
    //Iterate on the outgoing half edge of the end vertex of the input half edge
    Dcel::Vertex::OutgoingHalfEdgeIterator heit;
    for(heit = endVertex->outgoingHalfEdgeBegin(); heit != endVertex->outgoingHalfEdgeEnd(); ++heit){
        Dcel::HalfEdge* candidateTwin = *heit;
        
        //if an half edge has the end vertex equal to the start vertex of the input half egde it's his twin
        if(candidateTwin->getToVertex() == startVertex){
            candidateTwin->setTwin(he);
            he->setTwin(candidateTwin);
            
            //I've found the twin, no need to keep iterating
            break;
        }
    }
}

/**
 * @brief ConvexHullCreator::getVertices
 * Copies the vertices from the original dcel to the vertex vector
 */
void ConvexHullCreator::getVertices(){
    Dcel::VertexIterator vit;
    int i=0;
    for(vit = dcel->vertexBegin(); vit != dcel->vertexEnd(); ++vit){
        this->vertexVec[i] = *vit;
        i++;
    }
}
