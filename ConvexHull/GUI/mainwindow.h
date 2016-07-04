/**
 @author    Marco Livesu (marco.livesu@gmail.com)
 @copyright Marco Livesu 2014.
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>
#include <QToolBox>
#include <QFrame>
#include <QSignalMapper>
#include <QCheckBox>
#include <boost/bimap.hpp>

#include "lib/common/drawable_object.h"
#include "lib/common/bounding_box.h"

namespace Ui {
    class MainWindow;
}

/**
 * @brief La classe MainWindow è una classe che gestisce la canvas di QGLViewer e tutti i manager che vengono aggiunti ad essa.
 *
 * Gestisce in oltre una scrollArea avente le checkbox che gestiscono la visualizzazione di tutti gli oggetti presenti nella canvas.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

    public:

        explicit MainWindow(QWidget *parent = 0);
        ~MainWindow();

        void fitScene();
        void updateGlCanvas();
        void updateAndFit();
        void pushObj(DrawableObject * obj, std::string checkBoxName);
        void deleteObj(DrawableObject * obj);
        BoundingBox getFullBoundingBox();
        int getNumberVisibleObjects();
        void saveSnapshot();
        void drawAxis(bool);

        void setFullScreen(bool);
        void setBackgroundColor(const QColor &);

        //int addManager(QDockWidget *, Qt::DockWidgetArea position);
        int addManager(QFrame *f, std::string name, QToolBox *parent = nullptr);
        QFrame *getManager(unsigned int i);
        void renameManager(unsigned int i, std::string s);
        void setCurrentIndexToolBox(unsigned int i);

    signals:

    private slots:
        void checkBoxClicked(int i);

    private:

        // GUI
        //
        Ui::MainWindow  * ui;
        std::vector<QFrame *> managers;

        // Mesh Stack
        //
        QSignalMapper* checkBoxMapper;
        std::map<int, QCheckBox * > checkBoxes;
        boost::bimap<int, DrawableObject*> mapObjects;
        int nMeshes;


        bool first;
};

#endif // MAINWINDOW_H
