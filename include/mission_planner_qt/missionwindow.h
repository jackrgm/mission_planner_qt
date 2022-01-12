#ifndef MISSIONWINDOW_H
#define MISSIONWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <ros/ros.h>
#include "mission_planner_qt/backend.h"
#include "ui_missionwindow.h"

//#include "robotTask.h"
//#include "robotTasksMainWindow.h"

namespace mission_planner_qt {
  // TODO figure out how to optionally integrate with qt_robot_tasks
    // using preprocessing (e.g. #define use_qtrt) to alter code?
  //class MissionWindow : public qt_robot_tasks::robotTask {
  class MissionWindow : public QWidget
  {
    Q_OBJECT

    public:

      explicit MissionWindow(QWidget *parent = 0);

    /*
    MissionWindow(qt_robot_tasks::robotTasksMainWindow* mw, int pos, QWidget*
                  parent = 0);*/

    // ran when the user switches to this task in qt_robot_tasks
    //virtual bool activate(ros::NodeHandle* nh, const std::string& rosNamespace);

    // ran when the user switches to another task in qt_robot_tasks
    //virtual void deactivate();

    ~MissionWindow();

  private slots:
    void trailAdd();
    void trailClear();
    void trailDelete();
    void trailEdit();
    void trailLoad();
    void trailSave();
    void trailShiftUp();
    void trailShiftDown();
    void trailCapture();
    void trailCaptureAuto();

    void missionAbort();
    void missionPause();
    void missionStart();
    void missionReverse();

    void update();

  private:
    bool capturing = false;
    bool missionactive = false;

    // Backend calls to mission_planner classes are abstracted into a facade.
    Backend backend;

    // Used to regularly call update functions for 'live' information.
    QTimer* updateTimer;

    /*
     * Instead of UI elements being modified by slots throughout the code, a
     * single source (function) is used where every owned widget is enabled,
     * disabled, or edited in accordance with condition checks.
     *
     * For example, if a mission is active (missionactive == true), then all
     * controls related to trail management will be disabled, and the mission
     * controls (apart from starting a mission) will become enabled.
     */
    void widgetCheck();

    /*
     * Draws a 2D graphic of an active mission, called repeatedly in an update
     * loop for a live visual overview of key data. This includes the trail,
     * waypoints with colours showing whether they've been visited, and the
     * robot's position relative to the trail.
     */
    void drawMap();

    // Pointers needed for drawing the map (via the QGraphicsView approach).
    QGraphicsScene *scene;
    QGraphicsEllipseItem *waypoint;
    QGraphicsRectItem *robot;
    QGraphicsTextItem *txt;

    QBrush getVisitColour(int index);

    // Map Qt signals to Qt slots.
    void setSignalSlots();

    void updateTrail();

    Ui::missionwindow ui;
  };
}
#endif
