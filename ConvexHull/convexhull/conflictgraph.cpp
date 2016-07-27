#include "conflictgraph.h"

/**
 * @brief ConflictGraph::ConflictGraph
 * Performs the conflict graph initialization on the input dcel
 */
ConflictGraph::ConflictGraph(DrawableDcel *dcelP, const std::vector<Pointd> &vertexVecP)
{
    this->dcel = dcelP;
    this->pointVector = vertexVecP;

    //check the visibility of each face from each point
    for(std::vector<Pointd>::iterator pit = pointVector.begin(); pit != pointVector.end(); ++pit){

        for(Dcel::FaceIterator fit = dcel->faceBegin(); fit != dcel->faceEnd(); ++fit){

            if(checkVisibility(*fit, *pit)){
                insertInFconflict(*pit, *fit);
                insertInPconflict(*pit, *fit);
            }
            
        }
    }

}

hashlib::pool<Dcel::Face *> *ConflictGraph::getVisibleFaces(Pointd &vertex)
{
    hashlib::pool<Dcel::Face *> *faces =  this->Fconflict[vertex];

    if(faces == nullptr){
        faces = new hashlib::pool<Dcel::Face *>;
        this->Fconflict[vertex] = faces;
    }

    return new hashlib::pool<Dcel::Face *>(*faces);
}

hashlib::pool<Pointd> *ConflictGraph::getVisibleVertices(Dcel::Face *face)
{
    hashlib::pool<Pointd> *vertices = this->Pconflict[face];

    if(vertices == nullptr){
        vertices = new hashlib::pool<Pointd>;
        this->Pconflict[face] = vertices;
    }

    return new hashlib::pool<Pointd>(*vertices);
}


bool ConflictGraph::checkVisibility(Dcel::Face* face, const Pointd &vertex){
    Dcel::Vertex *v = *(face->incidentVertexBegin());

    return ((vertex - v->getCoordinate()).dot(getFaceNormalDirection(face)) > 0);
}

Pointd ConflictGraph::getFaceNormalDirection(Dcel::Face *face){
    Pointd vertices[3], vec1, vec2, norm;

    int i=0;
    for(Dcel::Face::IncidentVertexIterator vit = face->incidentVertexBegin(); vit != face->incidentVertexEnd(); ++vit){
        vertices[i] = (*vit)->getCoordinate();
        i++;
    }

    vec1 = vertices[1] - vertices[0];
    vec2 = vertices[2] - vertices[0];
    norm = vec1.cross(vec2);

    return norm;

}

void ConflictGraph::insertInFconflict(Pointd point, Dcel::Face *face)
{

    if(this->Fconflict[point] == nullptr){
        this->Fconflict[point] = new hashlib::pool<Dcel::Face*>;
    }

    this->Fconflict[point]->insert(face);
}

void ConflictGraph::insertInPconflict(Pointd point, Dcel::Face *face)
{
    if(this->Pconflict[face] == nullptr){
        this->Pconflict[face] = new hashlib::pool<Pointd>;
    }

    this->Pconflict[face]->insert(point);
}

void ConflictGraph::updateConflictGraph(Dcel::Face *face, hashlib::pool<Pointd>* candidateVertices)
{
    for(hashlib::pool<Pointd>::iterator pit = candidateVertices->begin(); pit != candidateVertices->end(); ++pit){
        if(checkVisibility(face, *pit)){
            insertInFconflict(*pit, face);
            insertInPconflict(*pit, face);
        }
    }
}

void ConflictGraph::deleteFaces(hashlib::pool<Dcel::Face *> *faces)
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


void ConflictGraph::deletePoint(Pointd &vertex)
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
}




