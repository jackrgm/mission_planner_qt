#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <QGraphicsTextItem>
#include "mission_planner_qt/missionwindow.h"

namespace mission_planner_qt {
//MissionWindow::MissionWindow(qt_robot_tasks::robotTasksMainWindow* mw,
  //int pos, QWidget* parent): qt_robot_tasks::robotTask(mw, 'r',
  //"&Mission Planner", pos, parent)
  MissionWindow::MissionWindow(QWidget* parent) : QWidget(parent)
  {
    ui.setupUi(this);
    this->setWindowTitle("Mission Qt");
    updateTimer = new QTimer(this);
    scene = new QGraphicsScene(this);
    setSignalSlots();
    ui.list_trail->clear();
    ui.map->setScene(scene);
    updateTimer->start(1000);
    widgetCheck();
  }

  MissionWindow::~MissionWindow()
  {
    backend.mission_abort();
    delete updateTimer;
  }

  // QTRT: Operations that are performed when the user switches to this task.
  //bool MissionWindow::activate(ros::NodeHandle* nh, const std::string& rosNamespace) {
  //}

  // QTRT: Operations that are performed when the user switches to another task.
  //void MissionWindow::deactivate() {
  //}

  void MissionWindow::setSignalSlots()
  {
    connect(ui.btn_add, SIGNAL(clicked()), this, SLOT(trailAdd()));
    connect(ui.btn_clear, SIGNAL(clicked()), this, SLOT(trailClear()));
    connect(ui.btn_delete, SIGNAL(clicked()), this, SLOT(trailDelete()));
    connect(ui.btn_edit, SIGNAL(clicked()), this, SLOT(trailEdit()));
    connect(ui.btn_trailload, SIGNAL(clicked()), this, SLOT(trailLoad()));
    connect(ui.btn_trailsave, SIGNAL(clicked()), this, SLOT(trailSave()));
    connect(ui.btn_shiftu, SIGNAL(clicked()), this, SLOT(trailShiftUp()));
    connect(ui.btn_shiftd, SIGNAL(clicked()), this, SLOT(trailShiftDown()));
    connect(ui.btn_capman, SIGNAL(clicked()), this, SLOT(trailCapture()));
    connect(ui.btn_capauto, SIGNAL(clicked()), this, SLOT(trailCaptureAuto()));
    connect(ui.list_trail, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(trailEdit()));

    connect(ui.mission_abort, SIGNAL(clicked()), this, SLOT(missionAbort()));
    connect(ui.mission_pause, SIGNAL(clicked()), this, SLOT(missionPause()));
    connect(ui.mission_start, SIGNAL(clicked()), this, SLOT(missionStart()));
    connect(ui.mission_reverse, SIGNAL(clicked()), this, SLOT(missionReverse()));

    connect(updateTimer, SIGNAL(timeout()), this, SLOT(update()));
  }

  void MissionWindow::widgetCheck() {
    if(!capturing && !missionactive && ui.list_trail->count() > 0)
    {
      ui.btn_clear->setEnabled(true);
      ui.btn_edit->setEnabled(true);
      ui.btn_shiftu->setEnabled(true);
      ui.btn_shiftd->setEnabled(true);
      ui.btn_delete->setEnabled(true);
      ui.btn_trailsave->setEnabled(true);
      ui.mission_start->setEnabled(true);
    }
    else
    {
      ui.btn_clear->setEnabled(false);
      ui.btn_edit->setEnabled(false);
      ui.btn_shiftu->setEnabled(false);
      ui.btn_shiftd->setEnabled(false);
      ui.btn_delete->setEnabled(false);
      ui.btn_trailsave->setEnabled(false);
      ui.mission_start->setEnabled(false);
    }

    if(!missionactive && !capturing)
    {
      ui.btn_capman->setEnabled(true);
    }
    else
    {
      ui.btn_capman->setEnabled(false);
    }

    if(missionactive)
    {
      ui.mission_pause->setEnabled(true);
      ui.mission_reverse->setEnabled(true);
      ui.mission_abort->setEnabled(true);
      ui.bar_progress->setEnabled(true);
      ui.lined_target->setEnabled(true);
      ui.btn_add->setEnabled(false);
      ui.btn_capauto->setEnabled(false);
    }
    else
    {
      ui.mission_pause->setEnabled(false);
      ui.mission_reverse->setEnabled(false);
      ui.mission_abort->setEnabled(false);
      ui.bar_progress->setEnabled(false);
      ui.bar_progress->setValue(0);
      ui.lined_target->setEnabled(false);
      ui.lined_target->setText("");
      ui.btn_add->setEnabled(true);
      ui.btn_capauto->setEnabled(true);
    }
  }

  // 15 decimal places is assumed to be good enough for latlon coords
  void MissionWindow::trailAdd()
  {
    bool ok = false;;
    double lat;
    double lon;
    double yaw;

    lat = QInputDialog::getDouble(this, "Add a navigation goal", "Enter "
      "latitude", 0.0, -2147483647, 2147483647, 15, &ok);

    if(ok)
    {
      lon = QInputDialog::getDouble(this, "Add a navigation goal",
        "Enter longitude", 0.0, -2147483647, 2147483647, 15, &ok);
    }

    if(ok)
    {
      yaw = QInputDialog::getDouble(this, "Add a navigation goal",
        "Enter orientation (yaw)", 0.0, -2147483647, 2147483647, 6, &ok);
    }

    if(ok)
    {
      if(ui.list_trail->currentRow() > -1)
      {
        backend.trailInsertAfter(ui.list_trail->currentRow(), lat, lon, yaw);
      }
      else
      {
        backend.trailAdd(lat, lon, yaw);
      }
      updateTrail();
    }
  }

  void MissionWindow::trailLoad() {
    QString dir = QFileDialog::getOpenFileName(this, "Load a trail file");
    if(dir != "")
    {
      backend.trailLoad(dir.toStdString());
      updateTrail();
    }
  }

  void MissionWindow::trailSave()
  {
    QString dir = QFileDialog::getSaveFileName(this, "Save a trail file");
    if(dir != "")
    {
      backend.trailSave(dir.toStdString());
    }
  }

  void MissionWindow::trailShiftUp() {
    if(ui.list_trail->currentRow() > 0)
    {
      backend.trailShift(ui.list_trail->currentRow(), true);
      updateTrail();
      ui.list_trail->setCurrentRow(ui.list_trail->currentRow() - 1);
    }
  }

  void MissionWindow::trailShiftDown() {
    if(ui.list_trail->currentRow() < (backend.trailSize() - 1))
    {
      backend.trailShift(ui.list_trail->currentRow(), false);
      updateTrail();
      ui.list_trail->setCurrentRow(ui.list_trail->currentRow() + 1);
    }
  }

  void MissionWindow::trailEdit()
  {
    bool ok = false;
    double lat;
    double lon;
    double yaw;

    if(ui.list_trail->currentRow() > -1 && !missionactive)
    {
      double lat = QInputDialog::getDouble(this, "Edit selected waypoint",
        "Enter a new latitude", 0.0, -2147483647, 2147483647, 15, &ok);

      if(ok)
      {
        double lon = QInputDialog::getDouble(this, "Edit selected waypoint",
          "Enter a new longitude", 0.0, -2147483647, 2147483647, 15, &ok);
      }

      if(ok)
      {
        double yaw = QInputDialog::getDouble(this, "Edit selected waypoint",
          "Enter a new orientation (yaw)", 0.0, -2147483647, 2147483647, 6, &ok);
      }

      if(ok)
      {
         backend.trailEdit(ui.list_trail->currentRow(), lat, lon, yaw);
         updateTrail();
      }
    }
  }

  void MissionWindow::trailDelete() {
    bool lastrow = false;
    if(ui.list_trail->currentRow() > -1)
    {
      if(ui.list_trail->currentRow() == (ui.list_trail->count() - 1))
      {
        lastrow = true;
      }
      backend.trailRemove(ui.list_trail->currentRow());
      updateTrail();

      if(lastrow)
      {
        ui.list_trail->setCurrentRow((ui.list_trail->count() - 1));
      }
    }
  }

  void MissionWindow::trailClear() {
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Clear trail contents",
      "Are you sure you want to clear the trail? All waypoints in the trail will "
      "be lost.", QMessageBox::Yes|QMessageBox::No);

    if (reply == QMessageBox::Yes) {
      backend.trailClear();
      updateTrail();
    } else {
    }
  }

  void MissionWindow::trailCapture() {
    int current = ui.list_trail->currentRow();
    int count = ui.list_trail->count();
    bool follow_tail = false;

    if(current == (count - 1))
    {
      follow_tail = true;
    }

    if(current < 0)
    {
      backend.capture((ui.list_trail->count() - 1));
    }
    else
    {
      backend.capture(current);
    }

    updateTrail();

    /*
     * Allows a chain of manual captures to sequentially add to the trail's tail,
     * but if the user has selected a row in the list that's not at the end, then
     * the sequential adding will start after the selected row.
     */
    if (follow_tail)
    {
      ui.list_trail->setCurrentRow((ui.list_trail->count() - 1));
    }
  }

  void MissionWindow::trailCaptureAuto()
  {
    if (!capturing)
    {
      ui.btn_capauto->setText("Capturing...");
      capturing = true;
    }
    else
    {
      ui.btn_capauto->setText("Auto capture");
      capturing = false;
      widgetCheck();
      backend.captureReset();
    }
  }

  void MissionWindow::missionStart()
  {
    backend.mission_start();
    missionactive = true;
    widgetCheck();
  }

  void MissionWindow::missionPause() {
    if(ui.mission_pause->text() == "Pause")
    {
      ui.mission_pause->setText("Paused");
      ui.mission_reverse->setEnabled(false);
    }
    else
    {
      ui.mission_pause->setText("Pause");
      ui.mission_reverse->setEnabled(true);
    }

    backend.mission_pause();
  }

  void MissionWindow::missionReverse() {
    if(ui.mission_reverse->text() == "Reverse")
    {
      ui.mission_reverse->setText("Reversing...");
    }
    else
    {
      ui.mission_reverse->setText("Reverse");
    }

    backend.mission_reverse();
  }

  void MissionWindow::missionAbort() {
      ui.mission_pause->setChecked(false);
      ui.mission_pause->setText("Pause");
      ui.mission_reverse->setChecked(false);
      ui.mission_reverse->setText("Reverse");
      backend.mission_abort();
      missionactive = false;
      widgetCheck();
  }

  // TODO use the distanceBetween method to get metres, then scale
  // TODO auto-scaling measurements / relative position (provided by Qt)?
  void MissionWindow::drawMap()
  {
    // Start fresh every loop.
    ui.map->scene()->clear();

    // TODO Get relevant data from backend for establishing parameters.
      // Need robot pos, waypoint pos's, total waypoints, waypoint (un)visits
    // TODO Need to have and call functions to calc positioning of items.
      // e.g. calc distance between one waypoint and another, etc.

    //QPainter painter(ui.map);
    //painter.begin(ui.map);

    //painter.setPen(Qt::black);

    //painter.drawLine(10, 10, 100, 100);

    if(!missionactive)
    {
      ui.map->scene()->addText("Mission inactive");
    }
    else
    {
      QPen outline(Qt::black);
      QBrush robotcolour(Qt::blue);
      outline.setWidth(1);

      std::vector<int> rangeclass_x = {-20, -40, -80, -120, -160};
      std::vector<int> rangeclass_y = {-20, -40};

      // NOTE North/West = -100, South/East = 100, Diagonals = -200/200
      // NOTE waypoint labels should be at waypoint_xpos+16, waypoint_ypos+12
      // NOTE UTM uses X/Y already - just scale it down to smaller values + center
      // TODO zoom if diagram > scene
      for(int i = 0; i < backend.trailSize(); i++) {
        //txt = scene->addText(QString::fromStdString(std::to_string(i)));
        //txt = setPos(a.at(i), b.at(i));
        //waypoint = scene->addEllipse(a.at(i),b.at(i), 50, 50, outline, waypoint_unvisited);

        // First waypoint
        if(i == 0)
        {
          // Draw in center.
          waypoint = scene->addEllipse(0, 0, 50, 50, outline, getVisitColour(i));
          txt = scene->addText(QString::fromStdString(std::to_string(i)));
          txt->setPos(16, 12);
        }
        else
        {
          // Calc relative distance and bearing from last waypoint, then draw.
          // TODO use UTM coordinates for establishing positions in the scene.
            // add all X / number of points = gives X of center point
            // same for Y
            // UTM X and Y needs to be swapped (may need to negate axes - lookup)
            // Represent orient (maybe an arrow or shape of waypoint)
          waypoint = scene->addEllipse(-100*i, 100*i, 50, 50, outline, getVisitColour(i));
          txt = scene->addText(QString::fromStdString(std::to_string(i)));
          txt->setPos((-100*i)+16, (100*i)+12);
        }
      }

      /*
       * Get robot's position and orientation, calculate relative distance and
       * bearing from target waypoint, then draw.
       */
      robot = scene->addRect(300, 100, 50, 50, outline, robotcolour);
    }
  }

  QBrush MissionWindow::getVisitColour(int index)
  {
    if(backend.waypointVisited(index))
    {
      QBrush visited(Qt::green);
      return visited;
    }
    else
    {
      QBrush unvisited(Qt::red);
      return unvisited;
    }
  }

  void MissionWindow::updateTrail()
  {
    int restored_selection = ui.list_trail->currentRow();

    ui.list_trail->clear();

    for(int i = 0; i < backend.trailSize(); i++)
    {
      QString index = QString::number(i+1) + ") ";
      QString waypoint = index + QString::fromStdString(backend.waypointAsString(i));
      ui.list_trail->addItem(waypoint);
    }

    if(ui.list_trail->count() == 1)
    {
      ui.list_trail->setCurrentRow(0);
    }
    else
    {
      ui.list_trail->setCurrentRow(restored_selection);
    }

    widgetCheck();
  }

  void MissionWindow::update()
  {
    backend.updateNode();
    backend.updateScheduler();

    // Reflect any newly auto-captured waypoints in the UI.
    if(capturing && backend.captureAuto())
    {
      updateTrail();
    }

    // Update the robot's current position in the interface.
    ui.lined_currentpos->setText(QString::fromStdString(backend.getRobotPos()));
    ui.lined_currentpos->setCursorPosition(0);

    // Update the scheduler's state in the interface.
    ui.lined_mstate->setText(QString::fromStdString(backend.getMissionState()));
    ui.lined_mstate->setCursorPosition(0);

    if(missionactive)
    {
      // Update the robot's current target waypoint in the interface.
      ui.lined_target->setText(QString::fromStdString(backend.getTarget()));
      ui.lined_target->setCursorPosition(0);

      // Update the progress bar.
      ui.bar_progress->setValue(backend.getProgress());
    }

    // Update the mission map.
    drawMap();
  }
}
