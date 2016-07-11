#include "conflictgraph.h"

/**
 * @brief ConflictGraph::ConflictGraph
 * Performs the conflict graph initialization on the input dcel
 */
ConflictGraph::ConflictGraph(DrawableDcel *dcelP, std::vector<Dcel::Vertex*> &vertexVecP)
{
    this->dcel = dcelP;
    this->vertexVec = vertexVecP;

    Eigen::MatrixXd matrix(4,4);

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
            }
            
            matrix(3, 0) = vertexVec[i]->getCoordinate().x();
            matrix(3, 1) = vertexVec[i]->getCoordinate().y();
            matrix(3, 2) = vertexVec[i]->getCoordinate().z();
            matrix(3, 3) = 1;
            
            double det = matrix.determinant();
            
            //se il determinante è positivo il punto è nello stesso semispazio della normale della faccia
            //la normale punta all'esterno, quindi il punto "vede" la faccia
            if(det > std::numeric_limits<double>::epsilon()){
                addToFaceMap(*fit, vertexVec[i]);

                //fai la stessa cosa per i vertici
            }
            
            
        }
    }


}

void ConflictGraph::addToFaceMap(Dcel::Face* face, Dcel::Vertex* vertexToAdd){
    std::list<Dcel::Vertex*> associatedVertexList = faceMap[face];

    associatedVertexList.push_front(vertexToAdd);
    faceMap[face] = associatedVertexList;
}

