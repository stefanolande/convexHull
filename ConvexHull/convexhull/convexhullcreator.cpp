#include "convexhullcreator.h"


ConvexHullCreator::ConvexHullCreator(DrawableDcel* dcel){
    this->dcel = dcel;
    this->vertexVec = std::vector<Pointd>(dcel->getNumberVertices());
}


void ConvexHullCreator::calculate(){

    int count = 0;
    
    //get the vertices list from the dcel
    getVertices();
    
    //clear the initial dcel
    dcel->reset();
    
    findValidPermutation();
    
    //create the inital tetrahedron
    createTetrahedron();

    //initialize the conflict graph
    ConflictGraph conflictGraph(dcel, vertexVec);

    //start the incremental cycle
    //for(std::vector<Dcel::Vertex*>::iterator it = vertexVec.begin(); it != vertexVec.end(); ++it){
    for(unsigned int i=4; i<vertexVec.size();i++){

        dcel->clearDebugCylinders();
        dcel->clearDebugSpheres();

        std::set<Dcel::Face*>* visibleFaces = conflictGraph.getVisibleFaces(vertexVec[i]);




        //if F_conflict(p_r) is not empty
        if(visibleFaces->size() > 0){

            //insert p_r in the DCEL
            //Dcel::Vertex* newVertex = dcel->addVertex(**it);
            Dcel::Vertex* newVertex = dcel->addVertex(vertexVec[i]);

            count++;

            //checkSanity();

            std::cout << "Facce visibili: " << visibleFaces->size() << std::endl;

            std::list<Dcel::HalfEdge*> horizon = getHorizon(visibleFaces);

            removeVisibleFaces(*visibleFaces);

            //std::cout << "#HE on horizon " << horizon.size() << std::endl;

            //add a new face from each vertex in the horizon to the new edge
            std::vector<Dcel::Face*> newFaces;

            for(std::list<Dcel::HalfEdge*>::iterator it = horizon.begin(); it != horizon.end(); ++it){
                Dcel::HalfEdge* halfEdge = *it;
                Dcel::Face* newFace = addFace(newVertex, halfEdge);
                newFaces.push_back(newFace);

                //conflictGraph.updateConflictGraph(newFace, candidateVisibleVerticesMap[halfEdge]);
                //conflictGraph.updateNaive(newFace);
                dcel->addDebugCylinder(halfEdge->getFromVertex()->getCoordinate(), halfEdge->getToVertex()->getCoordinate(), 0.01, QColor(255,0,0));
            }

            setTwins(newFaces);

        }

        //conflictGraph.deleteFaces(visibleFaces);
        //conflictGraph.deletePoint(vertexVec[i]);

        dcel->addDebugSphere(vertexVec[i], 0.01, QColor(255,0,0));

        if(count == 15){
            return;
        }

        conflictGraph = ConflictGraph(dcel, vertexVec);



    }


    std::cout << "#FACES " << dcel->getNumberFaces() << std::endl;
}

std::list<Dcel::HalfEdge*> ConvexHullCreator::getHorizon(std::set<Dcel::Face*>* visibleFaces){

    std::list<Dcel::HalfEdge*> horizon;
    Dcel::HalfEdge* first;

    //cerco un edge dell'orizzonte
    for(std::set<Dcel::Face*>::iterator fit = visibleFaces->begin(); fit != visibleFaces->end(); ++fit){

        Dcel::Face* faceToRemove = *fit;

        //remove all the half edges of the face
        for(Dcel::Face::IncidentHalfEdgeIterator iheit = faceToRemove->incidentHalfEdgeBegin(); iheit != faceToRemove->incidentHalfEdgeEnd(); ++iheit){
            Dcel::HalfEdge* heToRemove = *iheit;


            if(heToRemove->getTwin() != nullptr){
                //if the twin of an he has the incident face outside the visible faces, it is part of the horizon
                if(visibleFaces->count(heToRemove->getTwin()->getFace()) == 0){
                    first = heToRemove->getTwin();
                    break;
                }
            }
        }
    }

    //prendo il resto dell'orizzonte
    Dcel::HalfEdge *current, *next, *twin;
    Dcel::Face *incidentFace;

    current = first;
    horizon.push_front(first);

    do{
        next = current->getNext();
        twin = next->getTwin();
        incidentFace = twin->getFace();

        if(visibleFaces->count(incidentFace) > 1){
            std::cout << "COSE" << std::endl;
        }

        if(visibleFaces->count(incidentFace) == 1){
            horizon.push_back(next);
            current = next;
        } else {
            current = twin;
        }
    } while (first != current && first != current->getNext());

    return horizon;

}

void ConvexHullCreator::removeVisibleFaces(std::set<Dcel::Face*> &faceList){

    std::list<Dcel::Vertex*> vertexToRemove;

    for(std::set<Dcel::Face*>::iterator it = faceList.begin(); it != faceList.end(); ++it){

        Dcel::Face* face = *it;

        for(Dcel::Face::IncidentHalfEdgeIterator vit = face->incidentHalfEdgeBegin(); vit != face->incidentHalfEdgeEnd(); ++vit){

            Dcel::HalfEdge* he = *vit;

            Dcel::Vertex* start = he->getFromVertex();
            Dcel::Vertex* end = he->getToVertex();

            this->dcel->deleteHalfEdge(he);
            start->decrementCardinality();
            end->decrementCardinality();

            if(start->getCardinality() == 0){
                vertexToRemove.push_front(start);
            }
            if(end->getCardinality() == 0){
                vertexToRemove.push_front(end);
            }
        }


        this->dcel->deleteFace(face);
    }

    for(std::list<Dcel::Vertex*>::iterator it = vertexToRemove.begin(); it != vertexToRemove.end(); ++it){
        this->dcel->deleteVertex(*it);
    }
}

/**
 * @brief ConvexHullCreator::findValidPermutation
 * Does a random shuffle on the input vertices vector. The first four must not be coplanar to form a tetrahedron.
 */
void ConvexHullCreator::findValidPermutation(){
    //get a random permutation that starts with 4 non coplanar vertices
    bool coplanar = true;
    
    //std::srand(std::time(0));
    
    do{
        //calculate a random permutation of the vertices vector
        std::random_shuffle(this->vertexVec.begin(), this->vertexVec.end());
        
        //check if the first point are coplanar
        Eigen::Matrix4d matrix;
        for(int i=0; i<4; i++){
            matrix(i, 0) = this->vertexVec[i].x();
            matrix(i, 1) = this->vertexVec[i].y();
            matrix(i, 2) = this->vertexVec[i].z();
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
    Dcel::Vertex* a = this->dcel->addVertex(this->vertexVec[0]);
    Dcel::Vertex* b = this->dcel->addVertex(this->vertexVec[1]);
    Dcel::Vertex* c = this->dcel->addVertex(this->vertexVec[2]);
    Dcel::Vertex* d = this->dcel->addVertex(this->vertexVec[3]);
    
    Dcel::HalfEdge* he1In = this->dcel->addHalfEdge();
    Dcel::HalfEdge* he2In = this->dcel->addHalfEdge();
    Dcel::HalfEdge* he3In = this->dcel->addHalfEdge();
    
    he1In->setFromVertex(b);
    he1In->setToVertex(a);
    he1In->setNext(he3In);
    he1In->setPrev(he2In);
    a->incrementCardinality();
    b->incrementCardinality();
    
    he2In->setFromVertex(c);
    he2In->setToVertex(b);
    he2In->setNext(he1In);
    he2In->setPrev(he3In);
    c->incrementCardinality();
    b->incrementCardinality();
    
    he3In->setFromVertex(a);
    he3In->setToVertex(c);
    he3In->setNext(he2In);
    he3In->setPrev(he1In);
    a->incrementCardinality();
    c->incrementCardinality();
    
    Dcel::Face* face1 = this->dcel->addFace();
    face1->setOuterHalfEdge(he1In);
    he1In->setFace(face1);
    he2In->setFace(face1);
    he3In->setFace(face1);

    std::cout << "Aggiungo faccia " << face1->getId() << std::endl;
    
    addFaceForTetrahedron(d, he1In);
    addFaceForTetrahedron(d, he2In);
    addFaceForTetrahedron(d, he3In);

    //checkSanity();
    
}

/**
 * @brief ConvexHullCreator::addFace
 * Add a face connected to an existing vertex and an existing vertex. Used to attach new faces.
 * @param otherVertex
 * @param existingHe
 */
Dcel::Face* ConvexHullCreator::addFaceForTetrahedron(Dcel::Vertex* otherVertex, Dcel::HalfEdge* existingHe){
    Dcel::HalfEdge* he1 = this->dcel->addHalfEdge();
    Dcel::HalfEdge* he2 = this->dcel->addHalfEdge();
    Dcel::HalfEdge* he3 = this->dcel->addHalfEdge();
    
    Dcel::Vertex* startVertex = existingHe->getFromVertex();
    Dcel::Vertex* endVertex = existingHe->getToVertex();
    
    he1->setFromVertex(endVertex);
    endVertex->setIncidentHalfEdge(he1);
    he1->setToVertex(startVertex);
    he1->setNext(he2);
    he1->setPrev(he3);
    he1->setTwin(existingHe);
    existingHe->setTwin(he1);    
    endVertex->incrementCardinality();
    startVertex->incrementCardinality();
    
    he2->setFromVertex(startVertex);
    startVertex->setIncidentHalfEdge(he2);
    he2->setToVertex(otherVertex);
    he2->setNext(he3);
    he2->setPrev(he1);
    startVertex->incrementCardinality();
    otherVertex->incrementCardinality();

    //set the twin
    if(existingHe->getPrev()->getTwin() != nullptr){
        Dcel::HalfEdge* twin = existingHe->getPrev()->getTwin()->getPrev();
        he2->setTwin(twin);
        twin->setTwin(he2);
    }


    he3->setFromVertex(otherVertex);
    otherVertex->setIncidentHalfEdge(he3);
    he3->setToVertex(endVertex);
    he3->setNext(he1);
    he3->setPrev(he2);
    endVertex->incrementCardinality();
    otherVertex->incrementCardinality();

    //set the twin
    //adjustTwin(he3);
    if(existingHe->getNext()->getTwin() != nullptr){
        Dcel::HalfEdge* twin = existingHe->getNext()->getTwin()->getNext();
        he3->setTwin(twin);
        twin->setTwin(he3);
    }

    
    Dcel::Face* face = this->dcel->addFace();
    face->setOuterHalfEdge(he1);
    he1->setFace(face);
    he2->setFace(face);
    he3->setFace(face);

    //std::cout << "Aggiungo faccia " << face->getId() << std::endl;


    return face;
}

Dcel::Face* ConvexHullCreator::addFace(Dcel::Vertex* otherVertex, Dcel::HalfEdge* existingHe){
    Dcel::HalfEdge* he1 = this->dcel->addHalfEdge();
    Dcel::HalfEdge* he2 = this->dcel->addHalfEdge();
    Dcel::HalfEdge* he3 = this->dcel->addHalfEdge();

    Dcel::Vertex* startVertex = existingHe->getFromVertex();
    Dcel::Vertex* endVertex = existingHe->getToVertex();

    he1->setFromVertex(endVertex);
    endVertex->setIncidentHalfEdge(he1);
    he1->setToVertex(startVertex);
    he1->setNext(he2);
    he1->setPrev(he3);
    he1->setTwin(existingHe);
    existingHe->setTwin(he1);
    endVertex->incrementCardinality();
    startVertex->incrementCardinality();

    he2->setFromVertex(startVertex);
    startVertex->setIncidentHalfEdge(he2);
    he2->setToVertex(otherVertex);
    he2->setNext(he3);
    he2->setPrev(he1);
    startVertex->incrementCardinality();
    otherVertex->incrementCardinality();

    he3->setFromVertex(otherVertex);
    otherVertex->setIncidentHalfEdge(he3);
    he3->setToVertex(endVertex);
    he3->setNext(he1);
    he3->setPrev(he2);
    endVertex->incrementCardinality();
    otherVertex->incrementCardinality();

    Dcel::Face* face = this->dcel->addFace();
    face->setOuterHalfEdge(he1);
    he1->setFace(face);
    he2->setFace(face);
    he3->setFace(face);

    //std::cout << "Aggiungo faccia " << face->getId() << std::endl;

    return face;
}

/**
 * @brief ConvexHullCreator::adjustTwin
 * Given the input half hedge, the method finds its twin set it
 * @param he
 */
void ConvexHullCreator::setTwins(std::vector<Dcel::Face*> &faceList){

    std::vector<Dcel::HalfEdge*> he1Vector(faceList.size());
    std::vector<Dcel::HalfEdge*> he2Vector(faceList.size());
    std::vector<Dcel::HalfEdge*> he3Vector(faceList.size());

    for(unsigned int i=0; i<faceList.size(); i++){
        he1Vector[i] = faceList[i]->getOuterHalfEdge();
        he2Vector[i] = faceList[i]->getOuterHalfEdge()->getNext();
        he3Vector[i] = faceList[i]->getOuterHalfEdge()->getNext()->getNext();
    }

    for(unsigned int i=1; i<=faceList.size(); i++){
        he2Vector[i%faceList.size()]->setTwin(he3Vector[i-1]);
        he3Vector[i-1]->setTwin(he2Vector[i%faceList.size()]);
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
        this->vertexVec[i] = (*vit)->getCoordinate();
        i++;
    }
}

void ConvexHullCreator::checkSanity(){
    for (Dcel::HalfEdgeIterator heit = dcel->halfEdgeBegin(); heit != dcel->halfEdgeEnd(); ++heit){
        Dcel::HalfEdge* he = *heit;

        if(he->getTwin() == nullptr){
            std::cout << "CHECKSANITY: TWIN NULLO SU FACCIA " << he->getFace()->getId() << std::endl;
        }
    }
}

void ConvexHullCreator::removeHalfEdgeList(std::list<Dcel::HalfEdge*> &heList){
    for(std::list<Dcel::HalfEdge*>::iterator it = heList.begin(); it != heList.end(); ++it){
        dcel->deleteHalfEdge(*it);
    }
}
