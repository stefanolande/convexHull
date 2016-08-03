#include "convexhullcreator.h"

/**
 * @brief ConvexHullCreator::ConvexHullCreator Public contructor that initalizes the convex hull creator.
 * @param dcel DCEL containing the point used to create the convex hull
 */
ConvexHullCreator::ConvexHullCreator(DrawableDcel* dcel){
    this->dcel = dcel;
    this->pointVec = std::vector<Pointd>(dcel->getNumberVertices());
}

/**
 * @brief ConvexHullCreator::calculate Computes the convex hull, without updating the UI.
 */
void ConvexHullCreator::calculate(){  

    getVertices();
    
    //clear the initial dcel
    dcel->reset();
    
    findValidPermutation();
    
    createTetrahedron();

    conflictGraph = new ConflictGraph(dcel, pointVec);

    //start the incremental cycle
    for(unsigned int i=4; i<pointVec.size();i++){
        //get the faces visibile from the new vertex
        hashlib::pool<Dcel::Face*>* visibleFaces = conflictGraph->getVisibleFaces(pointVec[i]);

        if(visibleFaces->size() > 0){

            //insert new vertex in the DCEL
            Dcel::Vertex* newVertex = dcel->addVertex(pointVec[i]);

            //compute the horizon of the visible faces
            std::list<Dcel::HalfEdge*> horizon = getHorizon(visibleFaces);

            //compute the map of vertices that may be visible from the faces incident to the horizon half edges
            //(and their twins)
            hashlib::dict<Dcel::HalfEdge*, hashlib::pool<Pointd>*>  candidateVertexMap = getCandidateVerticesMap(horizon);

            //remove the visible faces from the conflict graph and from the dcel
            conflictGraph->deleteFaces(visibleFaces);
            removeVisibleFaces(visibleFaces);

            //add a new face from each vertex in the horizon to the new edge
            std::vector<Dcel::Face*> newFaces;

            for(std::list<Dcel::HalfEdge*>::iterator it = horizon.begin(); it != horizon.end(); ++it){
                Dcel::HalfEdge* halfEdge = *it;

                //create a new face and push it in the vector of new faces
                Dcel::Face* newFace = addFace(newVertex, halfEdge);
                newFaces.push_back(newFace);

                //update the conflict graph, checking the visibility for only the point candidate point
                //previously saved in the map
                conflictGraph->updateConflictGraph(newFace, candidateVertexMap[halfEdge]);
            }

            //set the twins using the ordered new faces vector
            setTwins(newFaces);
        }

        //finally, delete the new point from the conflict graph
        conflictGraph->deletePoint(pointVec[i]);
    }
}

/**
 * @brief ConvexHullCreator::calculate Computes the convex hull, updating the UI at each step.
 * @param mainWindow Window to update.
 */
void ConvexHullCreator::calculate(MainWindow *mainWindow){

    getVertices();

    //clear the initial dcel
    dcel->reset();

    findValidPermutation();

    createTetrahedron();

    conflictGraph = new ConflictGraph(dcel, pointVec);

    //start the incremental cycle
    for(unsigned int i=4; i<pointVec.size();i++){
        //get the faces visibile from the new vertex
        hashlib::pool<Dcel::Face*>* visibleFaces = conflictGraph->getVisibleFaces(pointVec[i]);

        if(visibleFaces->size() > 0){

            //insert new vertex in the DCEL
            Dcel::Vertex* newVertex = dcel->addVertex(pointVec[i]);

            //compute the horizon of the visible faces
            std::list<Dcel::HalfEdge*> horizon = getHorizon(visibleFaces);

            //compute the map of vertices that may be visible from the faces incident to the horizon half edges
            //(and their twins)
            hashlib::dict<Dcel::HalfEdge*, hashlib::pool<Pointd>*>  candidateVertexMap = getCandidateVerticesMap(horizon);

            //remove the visible faces from the conflict graph and from the dcel
            conflictGraph->deleteFaces(visibleFaces);
            removeVisibleFaces(visibleFaces);

            //add a new face from each vertex in the horizon to the new edge
            std::vector<Dcel::Face*> newFaces;

            for(std::list<Dcel::HalfEdge*>::iterator it = horizon.begin(); it != horizon.end(); ++it){
                Dcel::HalfEdge* halfEdge = *it;

                //create a new face and push it in the vector of new faces
                Dcel::Face* newFace = addFace(newVertex, halfEdge);
                newFaces.push_back(newFace);

                //update the conflict graph, checking the visibility for only the point candidate point
                //previously saved in the map
                conflictGraph->updateConflictGraph(newFace, candidateVertexMap[halfEdge]);
            }

            //set the twins using the ordered new faces vector
            setTwins(newFaces);

            //operations needed to render the dcel
            this->dcel->update();
            mainWindow->updateGlCanvas();
        }

        //finally, delete the new point from the conflict graph
        conflictGraph->deletePoint(pointVec[i]);
    }
}

/**
 * @brief ConvexHullCreator::getCandidateVertexMap Return a map that, for each half edge in the horizon, contains a set
 * of vertices that may be visible from a face that will be attached to the former.
 * @param horizon set containing the half edges in the horizon
 * @return Map associating a set of candidate visible vertices to an half edge
 */
hashlib::dict<Dcel::HalfEdge*, hashlib::pool<Pointd>*> ConvexHullCreator::getCandidateVerticesMap(std::list<Dcel::HalfEdge*> horizon){

    hashlib::dict<Dcel::HalfEdge *, hashlib::pool<Pointd>*> result;

    //for each half edge in the horizon
    for(std::list<Dcel::HalfEdge*>::iterator it = horizon.begin(); it != horizon.end(); ++it){
        Dcel::HalfEdge *halfEdge = *it;

        //get its and its twin incident faces
        Dcel::Face *face1 = halfEdge->getFace();
        Dcel::Face *face2 = halfEdge->getTwin()->getFace();

        //get the set of the vertices visible from the former faces
        hashlib::pool<Pointd> *conflict1, *conflict2;
        conflict1 = conflictGraph->getVisibleVertices(face1);
        conflict2 = conflictGraph->getVisibleVertices(face2);

        //merge the sets
        conflict1->insert(conflict2->begin(), conflict2->end());

        //associate the merged set to the horizon half edge
        result[halfEdge] = conflict1;
    }

    return result;

}

/**
 * @brief ConvexHullCreator::getHorizon Given a set of visible faces, returns an ordered list
 * of half edge in the orizon
 * @param visibleFaces set of visible faces
 * @return ordered list of half edge
 */
std::list<Dcel::HalfEdge*> ConvexHullCreator::getHorizon(hashlib::pool<Dcel::Face*>* visibleFaces){

    std::list<Dcel::HalfEdge*> horizon;
    Dcel::HalfEdge* first;

    //first, we look for an horizon half edge to start getting the complete ordered horizon
    bool found = false;

    hashlib::pool<Dcel::Face*>::iterator fit;
    for(fit = visibleFaces->begin(); fit != visibleFaces->end() && !found; ++fit){

        Dcel::Face* visibleFace = *fit;

        Dcel::Face::IncidentHalfEdgeIterator iheit;

        //we iterate on all the visible faces and stop we find the first horizon half edge
        for(iheit = visibleFace ->incidentHalfEdgeBegin(); iheit != visibleFace ->incidentHalfEdgeEnd() && !found; ++iheit){
            Dcel::HalfEdge* visibleHe = *iheit;


            if(visibleHe->getTwin() != nullptr){
                //if the twin of a visible half edge has the incident face outside the visible faces,
                //it is part of the horizon
                if(visibleFaces->count(visibleHe->getTwin()->getFace()) == 0){
                    first = visibleHe->getTwin();
                    found = true;
                }
            }
        }
    }

    //if we found the first half edge, we follow the faces to get all the horizon,
    //otherwise there DCEL is malformed
    if(found){
        Dcel::HalfEdge *current, *next, *twinOfNext;
        Dcel::Face *incidentFace;

        current = first;
        horizon.push_front(first);

        //we iterate until we get back to the first horizon half edge
        do{
            next = current->getNext();
            twinOfNext = next->getTwin();
            incidentFace = twinOfNext->getFace();

            //if the incident face of the twin is visible, we have an horizon half edge
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

/**
 * @brief ConvexHullCreator::removeVisibleFaces Removes the faces in the set
 * @param faceList set of faces to remove
 */
void ConvexHullCreator::removeVisibleFaces(hashlib::pool<Dcel::Face*> *faceList){

    for(hashlib::pool<Dcel::Face*>::iterator it = faceList->begin(); it != faceList->end(); ++it){

        Dcel::Face* face = *it;

        for(Dcel::Face::IncidentHalfEdgeIterator vit = face->incidentHalfEdgeBegin(); vit != face->incidentHalfEdgeEnd(); ++vit){

            Dcel::HalfEdge* he = *vit;

            Dcel::Vertex* start = he->getFromVertex();
            Dcel::Vertex* end = he->getToVertex();

            this->dcel->deleteHalfEdge(he);

            //decrement the cardinality of each vertex every time we remove an half edge
            start->decrementCardinality();
            end->decrementCardinality();

            //if the cardinality is 0, the vertex is disconnected and must be removed
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
    bool coplanar = true;

    std::random_device rd;
    std::mt19937 g(rd());

    //compute a random permutation of the vertices vector
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

    for(unsigned int i=0; i<4; i++){
        matrix(i, 0) = this->pointVec[i].x();
        matrix(i, 1) = this->pointVec[i].y();
        matrix(i, 2) = this->pointVec[i].z();
        matrix(i, 3) = 1;
    }

    double det = matrix.determinant();

    //if the determinant is negative, the fourth vertex is in the semi-space of the face normal.
    //We must orient the first face so the determinant is positive
    if (det < -std::numeric_limits<double>::epsilon()){
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

    } else {
        he1In->setFromVertex(a);
        he1In->setToVertex(b);
        he1In->setNext(he2In);
        he1In->setPrev(he3In);

        he2In->setFromVertex(b);
        he2In->setToVertex(c);
        he2In->setNext(he3In);
        he2In->setPrev(he1In);

        he3In->setFromVertex(c);
        he3In->setToVertex(a);
        he3In->setNext(he1In);
        he3In->setPrev(he2In);
    }

    a->setCardinality(2);
    b->setCardinality(2);
    c->setCardinality(2);

    Dcel::Face* face1 = this->dcel->addFace();
    face1->setOuterHalfEdge(he1In);
    he1In->setFace(face1);
    he2In->setFace(face1);
    he3In->setFace(face1);
    
    //add new faces connecting each half edge to the fourth vertex
    addFaceForTetrahedron(d, he1In);
    addFaceForTetrahedron(d, he2In);
    addFaceForTetrahedron(d, he3In);
}

/**
 * @brief ConvexHullCreator::addFaceForTetrahedron Adds a new face connecting the existing half edge to the vertex.
 * Used only when creatin the initial tetrahedron because it sets the twins in an immediate way, valid only in that case.
 * @param otherVertex Vertex in the new face
 * @param existingHe Half edge used to connect the new face
 */
void ConvexHullCreator::addFaceForTetrahedron(Dcel::Vertex* otherVertex, Dcel::HalfEdge* existingHe){
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

    //set the twin following the structure of the DCEL.
    //firstly they can be null but they will be all set when all the faces are created
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

    //set the twin following the structure of the DCEL.
    //firstly they can be null but they will be all set when all the faces are created
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
}

/**
 * @brief ConvexHullCreator::addFace Adds a new face connecting the existing half edge to the vertex.
 * Valid in each step of the algoritm
 * @param otherVertex Vertex in the new face
 * @param existingHe Half edge used to connect the new face
 * @return Pointer the the newly created face.
 */
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

    return face;
}

/**
 * @brief ConvexHullCreator::setTwins Set the twis of the faces in an ordered vector.
 * The faces MUST be ordered
 * @param faceList vector of ordered face
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
