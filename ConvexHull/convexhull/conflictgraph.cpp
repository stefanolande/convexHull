#include "conflictgraph.h"

/**
 * @brief ConflictGraph::ConflictGraph
 * Performs the conflict graph initialization on the input dcel
 */
ConflictGraph::ConflictGraph(DrawableDcel *dcelP, std::vector<Dcel::Vertex*> &vertexVecP)
{
    this->dcel = dcelP;
    this->vertexVec = vertexVecP;

    Eigen::Matrix4d matrix;

    int counter = 0;

    //check the visibility of each face from each point
    for(unsigned int i=4; i<vertexVec.size(); i++){

        for(Dcel::FaceIterator fit = dcel->faceBegin(); fit != dcel->faceEnd(); ++fit){

            //add the coordinates of the three vertices to the matrix
            int j=0;
            for(Dcel::Face::IncidentVertexIterator vit = (*fit)->incidentVertexBegin(); vit != (*fit)->incidentVertexEnd(); ++vit){
                Dcel::Vertex* v = *vit;
                matrix(j, 0) = v->getCoordinate().x();
                matrix(j, 1) = v->getCoordinate().y();
                matrix(j, 2) = v->getCoordinate().z();
                matrix(j, 3) = 1;
                j++;
            }
            
            matrix(3, 0) = vertexVec[i]->getCoordinate().x();
            matrix(3, 1) = vertexVec[i]->getCoordinate().y();
            matrix(3, 2) = vertexVec[i]->getCoordinate().z();
            matrix(3, 3) = 1;

            double det = matrix.determinant();

            //se il determinante è positivo il punto è nello stesso semispazio della normale della faccia
            //la normale punta all'esterno, quindi il punto "vede" la faccia
            if(det < -std::numeric_limits<double>::epsilon()){
                addToFaceMap(*fit, vertexVec[i]);
                addToPointMap(vertexVec[i], *fit);
                counter++;
            }
            
            
        }
    }
    std::cout << "Vertici fuori dal tetraedro " << counter << std::endl;

}

std::set<Dcel::Face *>* ConflictGraph::getVisibleFaces(Dcel::Vertex *vertex)
{
    if(pointMap[vertex] != nullptr){
        return pointMap[vertex];
    } else {
        return new std::set<Dcel::Face *>();
    }
}

std::set<Dcel::Vertex *>* ConflictGraph::getVisibleVertices(Dcel::Face *face)
{
    if(faceMap[face] != nullptr){
        return faceMap[face];
    } else {
        return new std::set<Dcel::Vertex *>();
    }
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

