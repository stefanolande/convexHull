#include "conflictgraph.h"

/**
 * @brief ConflictGraph::ConflictGraph Performs the conflict graph initialization
 * @param dcelP The DCEL that contains the faces to check
 * @param vertexVecP vector of point to check
 */
ConflictGraph::ConflictGraph(DrawableDcel *dcelP, const std::vector<Pointd> &vertexVecP)
{
    //check the visibility of each face from each point
    for(auto pit = vertexVecP.cbegin(); pit != vertexVecP.cend(); ++pit){

        for(Dcel::FaceIterator fit = dcelP->faceBegin(); fit != dcelP->faceEnd(); ++fit){

            if(checkVisibility(*fit, *pit)){
                insertInFconflict(*pit, *fit);
                insertInPconflict(*pit, *fit);
            }
            
        }
    }
}

/**
 * @brief ConflictGraph::getVisibleFaces Returns the faces visible from the input vertex
 * @param vertex
 * @return set of visible faces
 */
hashlib::pool<Dcel::Face *> *ConflictGraph::getVisibleFaces(Pointd &vertex)
{
    hashlib::pool<Dcel::Face *> *faces =  this->Fconflict[vertex];

    //if the face set associated to the vertex is null, construct an empty and add it to the map
    if(faces == nullptr){
        faces = new hashlib::pool<Dcel::Face *>;
        this->Fconflict[vertex] = faces;
    }

    //return a clone of the set for further manipulations
    return new hashlib::pool<Dcel::Face *>(*faces);
}

/**
 * @brief ConflictGraph::getVisibleVertices Returns the vertices visible from a face
 * @param face
 * @return set of visible points
 */
hashlib::pool<Pointd> *ConflictGraph::getVisibleVertices(Dcel::Face *face)
{
    hashlib::pool<Pointd> *vertices = this->Pconflict[face];

    //if the vertices set associated to the vertex is null, construct an empty and add it to the map
    if(vertices == nullptr){
        vertices = new hashlib::pool<Pointd>;
        this->Pconflict[face] = vertices;
    }

    //return a clone of the set for further manipulations
    return new hashlib::pool<Pointd>(*vertices);
}

/**
 * @brief ConflictGraph::checkVisibility Check if a vertex sees a face
 * @param face
 * @param vertex
 * @return true if the point sees the face
 */
bool ConflictGraph::checkVisibility(Dcel::Face* face, const Pointd &vertex){

    //get a vertex from the face
    Dcel::Vertex *v = *(face->incidentVertexBegin());

    //(vertex - v) is a vector joining the face and the vertex to check
    //if the dot product betweet it and the face normal is positive,
    //the vector lies in the same semi-space of the normal, implying that the vertex sees the face
    //I use this form instead the determinant of the 4x4 matrix for performance
    return ((vertex - v->getCoordinate()).dot(getFaceNormalDirection(face)) > std::numeric_limits<double>::epsilon());
}

/**
 * @brief ConflictGraph::getFaceNormalDirection Returns the direction of the face normal
 * @param face
 * @return a Pointd that represents the vector
 */
Pointd ConflictGraph::getFaceNormalDirection(Dcel::Face *face){
    Pointd vertices[3], vec1, vec2, dir;

    //get the vertices of the face
    int i=0;
    for(Dcel::Face::IncidentVertexIterator vit = face->incidentVertexBegin(); vit != face->incidentVertexEnd(); ++vit){
        vertices[i] = (*vit)->getCoordinate();
        i++;
    }

    //the normal vector is the cross product between two edge vectors of the face
    //I don't normalize because I only need the direction
    vec1 = vertices[1] - vertices[0];
    vec2 = vertices[2] - vertices[0];
    dir = vec1.cross(vec2);

    return dir;

}

/**
 * @brief ConflictGraph::insertInFconflict Inserts a face in the conflict list of a vertex
 * @param point Vertex of which you have to modify the conflict list
 * @param face face to add to the conflict list
 */
void ConflictGraph::insertInFconflict(Pointd point, Dcel::Face *face)
{
    //if the conflict list of the vertex does not exist
    //create and add it to the map
    if(this->Fconflict[point] == nullptr){
        this->Fconflict[point] = new hashlib::pool<Dcel::Face*>;
    }

    this->Fconflict[point]->insert(face);
}

/**
 * @brief ConflictGraph::insertInPconflict Insert the vertex in the conflict list of a face
 * @param point Vertex to add to the conflict list
 * @param face Face of which you have to modify the conflict list
 */
void ConflictGraph::insertInPconflict(Pointd point, Dcel::Face *face)
{
    //if the conflict list of the face does not exist
    //create and add it to the map
    if(this->Pconflict[face] == nullptr){
        this->Pconflict[face] = new hashlib::pool<Pointd>;
    }

    this->Pconflict[face]->insert(point);
}

/**
 * @brief ConflictGraph::updateConflictGraph Performs the update of the conflict graph for an iteration
 * of the convex hull algorithm. It checks the visibility of the face against the vertices in the input set.
 * @param face Face for the visibility check
 * @param candidateVertices Vertices that may see the face
 */
void ConflictGraph::updateConflictGraph(Dcel::Face *face, hashlib::pool<Pointd>* candidateVertices)
{
    for(hashlib::pool<Pointd>::iterator pit = candidateVertices->begin(); pit != candidateVertices->end(); ++pit){
        if(checkVisibility(face, *pit)){
            insertInFconflict(*pit, face);
            insertInPconflict(*pit, face);
        }
    }
}

/**
 * @brief ConflictGraph::deleteFaces Delete the faces in the set from the conflict graph
 * @param faces Set of faces to delete
 */
void ConflictGraph::deleteFaces(hashlib::pool<Dcel::Face *> *faces)
{   
    //iterate over the faces
    hashlib::pool<Dcel::Face*>::const_iterator fit;
    for(fit = faces->begin(); fit != faces->end(); ++fit){

        hashlib::pool<Pointd> *visiblePoints = this->Pconflict[*fit];

        //if the conflict list of the face exist,
        //delete it from the map
        if(visiblePoints != nullptr){
            this->Pconflict.erase(*fit);

            //we have to delete the face from the conflict list of each vertex
            //we iterate only on the vertices that see the face
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

/**
 * @brief ConflictGraph::deletePoint Delete a vertex from the conflict graph
 * @param vertex Vertex to delete
 */
void ConflictGraph::deletePoint(Pointd &vertex)
{
    hashlib::pool<Dcel::Face*> *visibleFace = this->Fconflict[vertex];

    //if the conflict list of the vertex exist,
    //delete it from the map
    if(visibleFace != nullptr){
        this->Fconflict.erase(vertex);

        //we have to delete the vertex from the conflict list of each face
        //we iterate only on the faces that see the vertex
        hashlib::pool<Dcel::Face *>::iterator fit;
        for(fit = visibleFace->begin(); fit != visibleFace->end(); ++fit){

            auto associatedSet = this->Pconflict[*fit];
            if(associatedSet != nullptr){
                associatedSet->erase(vertex);
            }
        }
    }
}




