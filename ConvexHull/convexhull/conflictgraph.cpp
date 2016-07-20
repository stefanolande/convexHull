#include "conflictgraph.h"

/**
 * @brief ConflictGraph::ConflictGraph
 * Performs the conflict graph initialization on the input dcel
 */
ConflictGraph::ConflictGraph(DrawableDcel *dcelP, const std::vector<Pointd> &vertexVecP)
{
    this->dcel = dcelP;

    //copy the
    std::vector<Pointd>::const_iterator it = vertexVecP.begin();
    it += 3;
    for(; it != vertexVecP.end(); ++it){
        this->pointList.push_back(*it);
    }

    //check the visibility of each face from each point
    for(std::list<Pointd>::iterator it = pointList.begin(); it != pointList.end(); ++it){

        for(Dcel::FaceIterator fit = dcel->faceBegin(); fit != dcel->faceEnd(); ++fit){

            if(checkVisibility(*fit, *it)){
                this->conflict.insert(std::make_pair(*it, *fit));
            }
            
            
        }
    }

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
}


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

    //se il determinante è negativo il punto è nello stesso semispazio della normale della faccia
    //la normale punta all'esterno, quindi il punto "vede" la faccia
    return (det < -std::numeric_limits<double>::epsilon());
}

void ConflictGraph::updateConflictGraph(Dcel::Face *face, std::set<Pointd>* candidateVertices)
{
    //int count=0;
    for(std::set<Pointd>::iterator it = candidateVertices->begin(); it != candidateVertices->end(); ++it){
        if(checkVisibility(face, *it)){
            this->conflict.insert(std::make_pair(*it, face));
        }
    }
}

void ConflictGraph::updateNaive(Dcel::Face* face){
    for(std::list<Pointd>::iterator it = pointList.begin(); it != pointList.end(); ++it){

        if(checkVisibility(face, *it)){
            this->conflict.insert(std::make_pair(*it, face));
        }
    }

}

void ConflictGraph::deleteFaces(std::unordered_set<Dcel::Face *> *faces)
{

    for(std::set<std::pair<Pointd, Dcel::Face*>>::iterator cit = this->conflict.begin(); cit != this->conflict.end();){
        if(faces->count(cit->second) == 1){
            this->conflict.erase(cit++);
        } else {
            ++cit;
        }
    }
}

void ConflictGraph::deletePoint(Pointd &vertex)
{
    for(std::set<std::pair<Pointd, Dcel::Face*>>::iterator cit = this->conflict.begin(); cit != this->conflict.end();){
        if((*cit).first == vertex){
            this->conflict.erase(cit++);
        } else {
            ++cit;
        }
    }

    pointList.remove(vertex);

}




