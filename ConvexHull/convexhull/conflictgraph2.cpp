#include "conflictgraph2.h"

/**
 * @brief ConflictGraph::ConflictGraph
 * Performs the conflict graph initialization on the input dcel
 */
ConflictGraph2::ConflictGraph2(DrawableDcel *dcelP, const std::vector<Pointd> &vertexVecP)
{
    this->dcel = dcelP;

    //copy the
    std::vector<Pointd>::const_iterator it = vertexVecP.begin();
    it += 3;
    for(; it != vertexVecP.end(); ++it){
        this->pointList.push_back(*it);
    }

    //check the visibility of each face from each point
    for(std::list<Pointd>::iterator pit = pointList.begin(); pit != pointList.end(); ++pit){

        for(Dcel::FaceIterator fit = dcel->faceBegin(); fit != dcel->faceEnd(); ++fit){

            if(checkVisibility(*fit, *pit)){
                insertInFconflict(*pit, *fit);
                insertInPconflict(*pit, *fit);
            }
            
        }
    }

}

hashlib::pool<Dcel::Face *> *ConflictGraph2::getVisibleFaces(Pointd &vertex)
{
    hashlib::pool<Dcel::Face *> *faces =  this->Fconflict[vertex];

    if(faces == nullptr){
        faces = new hashlib::pool<Dcel::Face *>;
        this->Fconflict[vertex] = faces;
    }

    return new hashlib::pool<Dcel::Face *>(*faces);
    //return faces;
}

hashlib::pool<Pointd> *ConflictGraph2::getVisibleVertices(Dcel::Face *face)
{
    hashlib::pool<Pointd> *vertices = this->Pconflict[face];

    if(vertices == nullptr){
        vertices = new hashlib::pool<Pointd>;
        this->Pconflict[face] = vertices;
    }

    return new hashlib::pool<Pointd>(*vertices);
    //return vertices;
}


bool ConflictGraph2::checkVisibility(Dcel::Face* face, const Pointd &vertex){
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

void ConflictGraph2::insertInFconflict(Pointd point, Dcel::Face *face)
{

    if(this->Fconflict[point] == nullptr){
        this->Fconflict[point] = new hashlib::pool<Dcel::Face*>;
    }

    this->Fconflict[point]->insert(face);
}

void ConflictGraph2::insertInPconflict(Pointd point, Dcel::Face *face)
{
    if(this->Pconflict[face] == nullptr){
        this->Pconflict[face] = new hashlib::pool<Pointd>;
    }

    this->Pconflict[face]->insert(point);
}

void ConflictGraph2::updateConflictGraph(Dcel::Face *face, hashlib::pool<Pointd>* candidateVertices)
{
    //int count=0;
    for(hashlib::pool<Pointd>::iterator pit = candidateVertices->begin(); pit != candidateVertices->end(); ++pit){
        if(checkVisibility(face, *pit)){
            insertInFconflict(*pit, face);
            insertInPconflict(*pit, face);
        }
    }
}

void ConflictGraph2::updateNaive(Dcel::Face* face){
    for(std::list<Pointd>::iterator pit = pointList.begin(); pit != pointList.end(); ++pit){

        if(checkVisibility(face, *pit)){
            insertInFconflict(*pit, face);
            insertInPconflict(*pit, face);
        }
    }

}

void ConflictGraph2::deleteFaces(hashlib::pool<Dcel::Face *> *faces)
{   
    hashlib::pool<Dcel::Face*>::const_iterator fit;
    for(fit = faces->begin(); fit != faces->end(); ++fit){

        hashlib::pool<Pointd> *visiblePoints = this->Pconflict[*fit];

        if(visiblePoints != nullptr){
            this->Pconflict.erase(*fit);

            hashlib::pool<Pointd>::iterator pit;
            for(pit = visiblePoints->begin(); pit != visiblePoints->end(); ++pit){

                auto associatedSet = this->Fconflict[*pit];
                if(associatedSet != nullptr){
                    associatedSet->erase(*fit);
                }
            }
        }
    }
}


void ConflictGraph2::deletePoint(Pointd &vertex)
{
    hashlib::pool<Dcel::Face*> *visibleFace = this->Fconflict[vertex];

    if(visibleFace != nullptr){
        this->Fconflict.erase(vertex);

        hashlib::pool<Dcel::Face *>::iterator fit;
        for(fit = visibleFace->begin(); fit != visibleFace->end(); ++fit){

            auto associatedSet = this->Pconflict[*fit];
            if(associatedSet != nullptr){
                associatedSet->erase(vertex);
            }
        }
    }

    pointList.remove(vertex);
}




