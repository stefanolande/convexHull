/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @author    Andreas Scalas (andreasscalas@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef DRAWABLE_DCEL_H
#define DRAWABLE_DCEL_H

#include "lib/common/pickable_object.h"
#include "lib/common/common.h"
#include "GUI/objects/sphere.h"
#include "GUI/objects/cylinder.h"
#include "dcel.h"

/**
 * \~Italian
 * @class DrawableDcel
 *
 * @brief Classe che eredita dalla classe Dcel e dall'interfaccia DrawalbleObject.
 * Rende visualizzabile in una canvas la Dcel.
 * Tuttavia, se si effettuano modifiche sulla Dcel (utilizzando quindi i metodi classici della Dcel),
 * queste non saranno immediatamente visibili sulla DrawableDcel. Per rendere visibili eventuali modifiche,
 * è necessario chiamare il metodo update().
 *
 * @warning il metodo update() non è particolarmente efficiente se chiamato ad ogni draw() (ossia ad ogni render
 * della dcel); è quindi caldamente consigliato chiamare il metodo update() solamente quando si vuole visualizzare
 * le modifiche effettuate sulla dcel.
 */
class DrawableDcel : public Dcel, public DrawableObject {
    public:

        DrawableDcel();
        DrawableDcel(const Dcel &d);
        virtual ~DrawableDcel();

        void init();
        void clear();

        // Implementation of the
        // DrawableObject interface
        //
        void  draw()         const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);

        void addDebugSphere(const Pointd& center, double radius, const QColor &color, int precision = 4);
        void clearDebugSpheres();
        void addDebugCylinder(const Pointd& a, const Pointd& b, double radius, const QColor color);
        void clearDebugCylinders();

        void update();

        // rendering options
        //
        void setWireframe(bool b);
        void setFlatShading();
        void setSmoothShading();
        void setPointsShading();
        void setWireframeColor(float r, float g, float b);
        void setWireframeWidth(float width);
        void setEnableVertexColor();
        void setEnableTriangleColor();

    protected:

        void renderPass() const;

        enum {
            DRAW_MESH           = 0b00000001,
            DRAW_POINTS         = 0b00000010,
            DRAW_FLAT           = 0b00000100,
            DRAW_SMOOTH         = 0b00001000,
            DRAW_WIREFRAME      = 0b00010000,
            DRAW_FACECOLOR      = 0b00100000,
            DRAW_VERTEXCOLOR    = 0b01000000,
        };

        int   drawMode; /** \~Italian @brief intero interpretato come stringa di bit rappresentante la modalità di visualizzazione della dcel*/
        int   wireframeWidth; /** \~Italian @brief dimensione del wireframe */
        float wireframeColor[3]; /** \~Italian @brief colore del wireframe (rgb float [0:1]) */

        std::vector<double> coords; /** \~Italian @brief vettore di coordinate usate per la visualizzazione: per aggiornare utilizzare metodo update() */
        std::vector<double> v_norm; /** \~Italian @brief vettore di normali ai vertici usate per la visualizzazione: per aggiornare utilizzare il metodo update() */
        std::vector<int> tris; /** \~Italian @brief vettore di triangoli (da considerare a triple di indici) usati per la visualizzazione: per aggiornare utilizzare il metodo update() */
        std::vector<float> colors; /** \~Italian @brief vettore di colori associati ai triangoli (da considerare come triple rgb float) usati per la visualizzazione: per aggiornare utilizzare il metodo update() */

        std::vector<unsigned int> triangles_face; /** \~Italian @brief vettore di mappatura triangoli->facce (ogni entrata ha posizione corrispondente a un terzo della posizione della tripla in tris e presenta l'identificativo di una faccia */

        typedef struct {
                Pointd center;
                double radius;
                QColor color;
                int precision;
        } Sphere;
        typedef struct {
                Pointd a;
                Pointd b;
                double radius;
                QColor color;
        } Cylinder;
        std::vector<Sphere> debugSpheres;
        std::vector<Cylinder> debugCylinders;
};

#endif // DRAWABLE_DCEL_H
