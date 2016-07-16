#include "conflictgraph.h"

/**
 * @brief ConflictGraph::ConflictGraph
 * Performs the conflict graph initialization on the input dcel
 */
ConflictGraph::ConflictGraph(DrawableDcel *dcelP, std::vector<Dcel::Vertex*> &vertexVecP)
{
    this->dcel = dcelP;
    this->vertexVec = vertexVecP;

    //check the visibility of each face from each point
    for(unsigned int i=4; i<vertexVec.size(); i++){

        for(Dcel::FaceIterator fit = dcel->faceBegin(); fit != dcel->faceEnd(); ++fit){

            if(checkVisibility(*fit, vertexVec[i])){
                //addToFaceMap(*fit, vertexVec[i]);
                //addToPointMap(vertexVec[i], *fit);

                this->conflict.insert(std::make_pair(vertexVec[i], *fit));
            }
            
            
        }
    }
}

std::set<Dcel::Face *>* ConflictGraph::getVisibleFaces(Dcel::Vertex *vertex)
{
    std::set<Dcel::Face *>* result = new std::set<Dcel::Face *>();

    for(std::set<std::pair<Dcel::Vertex*, Dcel::Face*>>::iterator it = this->conflict.begin(); it != this->conflict.end(); ++it){
        if(vertex == (*it).first){
            result->insert((*it).second);
        }
    }

    return result;

    /*if(pointMap[vertex] != nullptr){
        return pointMap[vertex];
    } else {
        return new std::set<Dcel::Face *>();
    }*/
}

std::set<Dcel::Vertex *>* ConflictGraph::getVisibleVertices(Dcel::Face *face)
{
    std::set<Dcel::Vertex *>* result = new std::set<Dcel::Vertex *>();

    for(std::set<std::pair<Dcel::Vertex*, Dcel::Face*>>::iterator it = this->conflict.begin(); it != this->conflict.end(); ++it){
        if(face == (*it).second){
            result->insert((*it).first);
        }
    }

    return result;

    /*if(faceMap[face] != nullptr){
        return faceMap[face];
    } else {
        return new std::set<Dcel::Vertex *>();
    }*/
}



void ConflictGraph::addToFaceMap(Dcel::Face* face, Dcel::Vertex* vertexToAdd){
    std::set<Dcel::Vertex*>* associatedVertexSet = faceMap[face];

    if(associatedVertexSet == nullptr){
        associatedVertexSet = new std::set<Dcel::Vertex*>();
        faceMap[face] = associatedVertexSet;
    }

    associatedVertexSet->insert(vertexToAdd);
}

void ConflictGraph::addToPointMap(Dcel::Vertex* vertex, Dcel::Face* faceToAdd){
    std::set<Dcel::Face*>* associatedFaceSet = pointMap[vertex];

    if(associatedFaceSet == nullptr){
        associatedFaceSet = new std::set<Dcel::Face*>();
        pointMap[vertex] = associatedFaceSet;
    }

    associatedFaceSet->insert(faceToAdd);
}

bool ConflictGraph::checkVisibility(Dcel::Face* face, Dcel::Vertex* vertex){
    Eigen::Matrix4d matrix;
    //add the coordinates of the three vertices to the matrix
    int j=0;
    for(Dcel::Face::IncidentVertexIterator vit = face->incidentVertexBegin(); vit != face->incidentVertexEnd(); ++vit){
        Dcel::Vertex* v = *vit;
        matrix(j, 0) = (*vit)->getCoordinate().x();
        matrix(j, 1) = v->getCoordinate().y();
        matrix(j, 2) = v->getCoordinate().z();
        matrix(j, 3) = 1;
        j++;
    }

    matrix(3, 0) = vertex->getCoordinate().x();
    matrix(3, 1) = vertex->getCoordinate().y();
    matrix(3, 2) = vertex->getCoordinate().z();
    matrix(3, 3) = 1;

    double det = matrix.determinant();

    //se il determinante è positivo il punto è nello stesso semispazio della normale della faccia
    //la normale punta all'esterno, quindi il punto "vede" la faccia
    return (det < -std::numeric_limits<double>::epsilon());
}

void ConflictGraph::updateConflictGraph(Dcel::Face *face, std::set<Dcel::Vertex*> *candidateVertices)
{
    //int count=0;
    for(std::set<Dcel::Vertex*>::iterator it = candidateVertices->begin(); it != candidateVertices->end(); ++it){
        if(checkVisibility(face, *it)){
            //addToFaceMap(face, *it);
            //addToPointMap(*it, face);
            //count++;
            this->conflict.insert(std::make_pair(*it, face));
        }
    }
    //std::cout << "Added " << count << " links" << std::endl;
}

void ConflictGraph::deleteFaces(std::set<Dcel::Face *> *faces)
{
    for(std::set<Dcel::Face *>::iterator it = faces->begin(); it != faces->end(); ++it){

        for(std::set<std::pair<Dcel::Vertex*, Dcel::Face*>>::iterator cit = this->conflict.begin(); cit != this->conflict.end(); ++cit){
            if((*cit).second == *it){
                this->conflict.erase(cit);
            }
        }

        /*this->faceMap.erase(*it);

        for(std::map<Dcel::Vertex*, std::set<Dcel::Face*>*>::iterator pit = pointMap.begin(); pit != pointMap.end(); ++pit){
            if(pit->second != nullptr){
                pit->second->erase(*it);
            }
        }*/
    }
}

void ConflictGraph::deletePoint(Dcel::Vertex *vertex)
{
    for(std::set<std::pair<Dcel::Vertex*, Dcel::Face*>>::iterator cit = this->conflict.begin(); cit != this->conflict.end(); ++cit){
        if((*cit).first == vertex){
            this->conflict.erase(cit);
        }
    }
}

