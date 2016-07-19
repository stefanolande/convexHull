#include "conflictgraph.h"

/**
 * @brief ConflictGraph::ConflictGraph
 * Performs the conflict graph initialization on the input dcel
 */
ConflictGraph::ConflictGraph(DrawableDcel *dcelP, std::vector<Pointd> vertexVecP)
{
    this->dcel = dcelP;
    this->vertexVec = vertexVecP;

    std::cout << "Starting conflict graph initialization" << std::endl;

    vertexVecP.erase(vertexVecP.begin());
    vertexVecP.erase(vertexVecP.begin()+1);
    vertexVecP.erase(vertexVecP.begin()+2);
    vertexVecP.erase(vertexVecP.begin()+3);


    //check the visibility of each face from each point
    for(unsigned int i=0; i<vertexVec.size(); i++){

        for(Dcel::FaceIterator fit = dcel->faceBegin(); fit != dcel->faceEnd(); ++fit){

            if(checkVisibility(*fit, vertexVec[i])){
                //addToFaceMap(*fit, vertexVec[i]);
                //addToPointMap(vertexVec[i], *fit);

                this->conflict.insert(std::make_pair(vertexVec[i], *fit));
            }
            
            
        }
    }

    std::cout << "Conflict graph initialization completed" << std::endl;
}

std::unordered_set<Dcel::Face *>* ConflictGraph::getVisibleFaces(Pointd &vertex)
{
    std::unordered_set<Dcel::Face *>* result = new std::unordered_set<Dcel::Face *>();

    for(std::set<std::pair<Pointd, Dcel::Face*>>::iterator it = this->conflict.begin(); it != this->conflict.end(); ++it){
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

std::set<Pointd> *ConflictGraph::getVisibleVertices(Dcel::Face *face)
{
    std::set<Pointd >* result = new std::set<Pointd >();

    for(std::set<std::pair<Pointd, Dcel::Face*>>::iterator it = this->conflict.begin(); it != this->conflict.end(); ++it){
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



/*void ConflictGraph::addToFaceMap(Dcel::Face* face, const Pointd &vertexToAdd){
    std::set<Dcel::Vertex*>* associatedVertexSet = faceMap[face];

    if(associatedVertexSet == nullptr){
        associatedVertexSet = new std::set<Dcel::Vertex*>();
        faceMap[face] = associatedVertexSet;
    }

    associatedVertexSet->insert(vertexToAdd);
}*/

/*void ConflictGraph::addToPointMap(const Pointd &vertex, Dcel::Face* faceToAdd){
    std::set<Dcel::Face*>* associatedFaceSet = pointMap[vertex];

    if(associatedFaceSet == nullptr){
        associatedFaceSet = new std::set<Dcel::Face*>();
        pointMap[vertex] = associatedFaceSet;
    }

    associatedFaceSet->insert(faceToAdd);
}*/

bool ConflictGraph::checkVisibility(Dcel::Face* face, const Pointd &vertex){
    Eigen::Matrix4d matrix;
    //add the coordinates of the three vertices to the matrix
    int j=0;
    for(Dcel::Face::IncidentVertexIterator vit = face->incidentVertexBegin(); vit != face->incidentVertexEnd(); ++vit){
        Dcel::Vertex* v = *vit;
        matrix(j, 0) = v->getCoordinate().x();
        matrix(j, 1) = v->getCoordinate().y();
        matrix(j, 2) = v->getCoordinate().z();
        matrix(j, 3) = 1;
        j++;
    }

    matrix(3, 0) = vertex.x();
    matrix(3, 1) = vertex.y();
    matrix(3, 2) = vertex.z();
    matrix(3, 3) = 1;

    double det = matrix.determinant();

    //se il determinante è positivo il punto è nello stesso semispazio della normale della faccia
    //la normale punta all'esterno, quindi il punto "vede" la faccia
    return (det < -std::numeric_limits<double>::epsilon());
}

void ConflictGraph::updateConflictGraph(Dcel::Face *face, const std::set<Pointd *> &candidateVertices)
{
    //int count=0;
    for(std::set<Pointd*>::iterator it = candidateVertices.begin(); it != candidateVertices.end(); ++it){
        if(checkVisibility(face, **it)){
            //addToFaceMap(face, *it);
            //addToPointMap(*it, face);
            //count++;
            this->conflict.insert(std::make_pair(**it, face));
        }
    }
    //std::cout << "Added " << count << " links" << std::endl;
}

void ConflictGraph::updateNaive(Dcel::Face* face){
    for(unsigned int i=0; i<vertexVec.size(); i++){
        if(checkVisibility(face, vertexVec[i])){
            //addToFaceMap(face, *it);
            //addToPointMap(*it, face);
            //count++;
            this->conflict.insert(std::make_pair(vertexVec[i], face));
        }
    }

}

void ConflictGraph::deleteFaces(std::unordered_set<Dcel::Face *> *faces)
{
    for(std::unordered_set<Dcel::Face *>::iterator it = faces->begin(); it != faces->end(); ++it){

        for(std::set<std::pair<Pointd, Dcel::Face*>>::iterator cit = this->conflict.begin(); cit != this->conflict.end(); ++cit){
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

void ConflictGraph::deletePoint(Pointd &vertex)
{
    for(std::set<std::pair<Pointd, Dcel::Face*>>::iterator cit = this->conflict.begin(); cit != this->conflict.end(); ++cit){
        if((*cit).first == vertex){
            this->conflict.erase(cit);
        }
    }

    vertexVec.erase(std::remove(vertexVec.begin(), vertexVec.end(), vertex), vertexVec.end());

}




