#include "convexhullcreator.h"

ConvexHullCreator::ConvexHullCreator(DrawableDcel* dcel){
    this->dcel = dcel;
    this->pointVec = std::vector<Pointd>(dcel->getNumberVertices());
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
    conflictGraph = new ConflictGraph(dcel, pointVec);

    //start the incremental cycle
    //for(std::vector<Dcel::Vertex*>::iterator it = vertexVec.begin(); it != vertexVec.end(); ++it){
    for(unsigned int i=4; i<pointVec.size();i++){

        //dcel->clearDebugCylinders();
        //dcel->clearDebugSpheres();

        hashlib::pool<Dcel::Face*>* visibleFaces = conflictGraph->getVisibleFaces(pointVec[i]);

        //if F_conflict(p_r) is not empty
        if(visibleFaces->size() > 0){

            //insert p_r in the DCEL
            //Dcel::Vertex* newVertex = dcel->addVertex(**it);
            Dcel::Vertex* newVertex = dcel->addVertex(pointVec[i]);

            count++;

            //checkSanity();

            //std::cout << "Facce visibili: " << visibleFaces->size() << std::endl;

            std::list<Dcel::HalfEdge*> horizon = getHorizon(visibleFaces);
            hashlib::dict<Dcel::HalfEdge*, hashlib::pool<Pointd>*>  candidateVertexMap = getCandidateVertexMap(horizon);

            conflictGraph->deleteFaces(visibleFaces);
            removeVisibleFaces(visibleFaces);

            //std::cout << "#HE on horizon " << horizon.size() << std::endl;

            //add a new face from each vertex in the horizon to the new edge
            std::vector<Dcel::Face*> newFaces;

            for(std::list<Dcel::HalfEdge*>::iterator it = horizon.begin(); it != horizon.end(); ++it){
                Dcel::HalfEdge* halfEdge = *it;
                Dcel::Face* newFace = addFace(newVertex, halfEdge);
                newFaces.push_back(newFace);

                conflictGraph->updateConflictGraph(newFace, candidateVertexMap[halfEdge]);

                //conflictGraph->updateNaive(newFace);
                //dcel->addDebugCylinder(halfEdge->getFromVertex()->getCoordinate(), halfEdge->getToVertex()->getCoordinate(), 0.005, QColor(255,0,0));
            }

            setTwins(newFaces);

            //conflictGraph = new ConflictGraph(dcel, pointVec);
            //dcel->addDebugSphere(pointVec[i], 0.005, QColor(255,0,0));
            count++;

        }

        conflictGraph->deletePoint(pointVec[i]);

        /*if(count > 100){
            //return;
        }*/

    }
}

hashlib::dict<Dcel::HalfEdge*, hashlib::pool<Pointd>*> ConvexHullCreator::getCandidateVertexMap(std::list<Dcel::HalfEdge*> horizon){

    hashlib::dict<Dcel::HalfEdge *, hashlib::pool<Pointd>*> result;

    for(std::list<Dcel::HalfEdge*>::iterator it = horizon.begin(); it != horizon.end(); ++it){
        Dcel::HalfEdge *halfEdge = *it;
        Dcel::Face *face1 = halfEdge->getFace();
        Dcel::Face *face2 = halfEdge->getTwin()->getFace();

        hashlib::pool<Pointd> *conflict1, *conflict2;
        conflict1 = conflictGraph->getVisibleVertices(face1);
        conflict2 = conflictGraph->getVisibleVertices(face2);

        //merge the sets
        conflict1->insert(conflict2->begin(), conflict2->end());

        result[halfEdge] = conflict1;
    }

    return result;

}

std::list<Dcel::HalfEdge*> ConvexHullCreator::getHorizon(hashlib::pool<Dcel::Face*>* visibleFaces){

    std::list<Dcel::HalfEdge*> horizon;
    Dcel::HalfEdge* first;

    //cerco un edge dell'orizzonte
    bool found = false;
    hashlib::pool<Dcel::Face*>::iterator fit;
    for(fit = visibleFaces->begin(); fit != visibleFaces->end() && !found; ++fit){

        Dcel::Face* visibleFace = *fit;

        //remove all the half edges of the face
        Dcel::Face::IncidentHalfEdgeIterator iheit;
        for(iheit = visibleFace ->incidentHalfEdgeBegin(); iheit != visibleFace ->incidentHalfEdgeEnd() && !found; ++iheit){
            Dcel::HalfEdge* heToRemove = *iheit;


            if(heToRemove->getTwin() != nullptr){
                //if the twin of an he has the incident face outside the visible faces, it is part of the horizon
                if(visibleFaces->count(heToRemove->getTwin()->getFace()) == 0){
                    first = heToRemove->getTwin();
                    found = true;
                }
            }
        }
    }

    if(first != nullptr){
        //prendo il resto dell'orizzonte
        Dcel::HalfEdge *current, *next, *twinOfNext;
        Dcel::Face *incidentFace;

        current = first;
        horizon.push_front(first);

        do{
            next = current->getNext();
            twinOfNext = next->getTwin();
            incidentFace = twinOfNext->getFace();

            if(visibleFaces->count(incidentFace) == 1){
                horizon.push_back(next);
                current = next;
            } else {
                current = twinOfNext;
            }
        } while (first != current && first != current->getNext());
    } else {
        throw std::runtime_error("Error in visible faces!");
    }

    return horizon;

}

void ConvexHullCreator::removeVisibleFaces(hashlib::pool<Dcel::Face*> *faceList){

    for(hashlib::pool<Dcel::Face*>::iterator it = faceList->begin(); it != faceList->end(); ++it){

        Dcel::Face* face = *it;

        for(Dcel::Face::IncidentHalfEdgeIterator vit = face->incidentHalfEdgeBegin(); vit != face->incidentHalfEdgeEnd(); ++vit){

            Dcel::HalfEdge* he = *vit;

            Dcel::Vertex* start = he->getFromVertex();
            Dcel::Vertex* end = he->getToVertex();

            this->dcel->deleteHalfEdge(he);
            start->decrementCardinality();
            end->decrementCardinality();

            if(start->getCardinality() == 0){
                this->dcel->deleteVertex(start);
            }
            if(end->getCardinality() == 0){
                this->dcel->deleteVertex(end);
            }
        }


        this->dcel->deleteFace(face);
    }
}

/**
 * @brief ConvexHullCreator::findValidPermutation
 * Does a random shuffle on the input vertices vector. The first four must not be coplanar to form a tetrahedron.
 */
void ConvexHullCreator::findValidPermutation(){
    //get a random permutation that starts with 4 non coplanar vertices
    bool coplanar = true;
    
    std::srand(std::time(0));

    std::random_device rd;
    std::mt19937 g(rd());

    //calculate a random permutation of the vertices vector
    std::shuffle(this->pointVec.begin(), this->pointVec.end(), g);

    
    do{
        //check if the first point are coplanar
        Eigen::Matrix4d matrix;
        for(int i=0; i<4; i++){
            matrix(i, 0) = this->pointVec[i].x();
            matrix(i, 1) = this->pointVec[i].y();
            matrix(i, 2) = this->pointVec[i].z();
            matrix(i, 3) = 1;
        }
        
        double det = matrix.determinant();

        //check if the determinant is 0 +- epsilon
        coplanar = det > -std::numeric_limits<double>::epsilon() && det < std::numeric_limits<double>::epsilon();

        if(coplanar){
            //swap the fourth element with another in the range (5, size)
            int index = (std::rand()+4) % (pointVec.size()-4);
            Pointd temp = pointVec[3];
            pointVec[3] = pointVec[index];
            pointVec[index] = pointVec[3];
        }
        
    } while(coplanar);
}

/**
 * @brief ConvexHullCreator::createTetrahedron
 * This method create the tetrahedon needed to start the incremental step of the convex hull algortihm
 */
void ConvexHullCreator::createTetrahedron(){
    //add the 4 point
    Dcel::Vertex* a = this->dcel->addVertex(this->pointVec[0]);
    Dcel::Vertex* b = this->dcel->addVertex(this->pointVec[1]);
    Dcel::Vertex* c = this->dcel->addVertex(this->pointVec[2]);
    Dcel::Vertex* d = this->dcel->addVertex(this->pointVec[3]);
    
    Dcel::HalfEdge* he1In = this->dcel->addHalfEdge();
    Dcel::HalfEdge* he2In = this->dcel->addHalfEdge();
    Dcel::HalfEdge* he3In = this->dcel->addHalfEdge();

    //check the orientation of the 4th point wrt the others
    //to determine the base face orientation

    Eigen::Matrix4d matrix;

    matrix(0, 0) = a->getCoordinate().x();
    matrix(0, 1) = a->getCoordinate().y();
    matrix(0, 2) = a->getCoordinate().z();
    matrix(0, 3) = 1;

    matrix(1, 0) = b->getCoordinate().x();
    matrix(1, 1) = b->getCoordinate().y();
    matrix(1, 2) = b->getCoordinate().z();
    matrix(1, 3) = 1;

    matrix(2, 0) = c->getCoordinate().x();
    matrix(2, 1) = c->getCoordinate().y();
    matrix(2, 2) = c->getCoordinate().z();
    matrix(2, 3) = 1;

    matrix(3, 0) = d->getCoordinate().x();
    matrix(3, 1) = d->getCoordinate().y();
    matrix(3, 2) = d->getCoordinate().z();
    matrix(3, 3) = 1;

    double det = matrix.determinant();

    //se il determinante è negativo il punto è nello stesso semispazio della normale della faccia
    //la normale punta all'esterno, quindi il punto "vede" la faccia
    if (det < -std::numeric_limits<double>::epsilon()){

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

    } else {

        he1In->setFromVertex(a);
        he1In->setToVertex(b);
        he1In->setNext(he2In);
        he1In->setPrev(he3In);
        a->incrementCardinality();
        b->incrementCardinality();

        he2In->setFromVertex(b);
        he2In->setToVertex(c);
        he2In->setNext(he3In);
        he2In->setPrev(he1In);
        c->incrementCardinality();
        b->incrementCardinality();

        he3In->setFromVertex(c);
        he3In->setToVertex(a);
        he3In->setNext(he1In);
        he3In->setPrev(he2In);
        a->incrementCardinality();
        c->incrementCardinality();

    }

    Dcel::Face* face1 = this->dcel->addFace();
    face1->setOuterHalfEdge(he1In);
    he1In->setFace(face1);
    he2In->setFace(face1);
    he3In->setFace(face1);
    
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
        this->pointVec[i] = (*vit)->getCoordinate();
        i++;
    }
}
