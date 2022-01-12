#ifndef BACKEND_H
#define BACKEND_H

#include <string>
#include <iostream>
#include <sstream>
#include "mission_planner/node.h"
#include "mission_planner/recorder.h"
#include "mission_planner/scheduler.h"
#include "mission_planner/trail.h"

namespace mission_planner_qt {
  /*
   * A facade-style class that abstracts the logic which calls the backend
   * (from mission_planner) for the UI.
   */
  class Backend
  {
    public:

      void updateNode();


      void capture(int index);


      bool captureAuto();


      void captureReset();
      void updateScheduler();
      std::string waypointAsString(int index);
      int trailSize();
      void trailAdd(double lat, double lon, double yaw);
      void trailInsertAfter(int index, double lat, double lon, double yaw);
      void trailShift(int index, bool dir);
      void trailEdit(int index, double lat, double lon, double orient);
      void trailRemove(int index);
      void trailClear();

      int trailLoad(std::string dir);
      int trailSave(std::string dir);

      void mission_start();
      void mission_pause();
      void mission_reverse();
      void mission_abort();

      std::string getTarget();
      int getProgress();

      std::string getRobotPos();

      bool waypointVisited(int index);

      std::string getMissionState();

    private:
      mission_planner::Node node;
      mission_planner::Recorder recorder;
      mission_planner::Scheduler scheduler;
      mission_planner::Trail trail;
  };
}
#endif
