#include "mission_planner_qt/backend.h"

namespace mission_planner_qt {
  void Backend::updateNode()
  {
    node.update();
  }

  void Backend::updateScheduler()
  {
    scheduler.process(node);
  }

  std::string Backend::waypointAsString(int index)
  {
    std::ostringstream ostrings;
    std::string output;

    ostrings << std::setprecision(std::numeric_limits<double>::digits10);
    ostrings << trail.getLat(index) << ", ";
    ostrings << trail.getLon(index) << ", ";
    ostrings << trail.getOrient(index);

    output = ostrings.str();

    return output;
  }

  int Backend::trailSize()
  {
    return trail.getSize();
  }

  void Backend::trailAdd(double lat, double lon, double yaw)
  {
    trail.add(lat, lon, yaw);
  }

  void Backend::trailInsertAfter(int index, double lat, double lon, double yaw)
  {
    trail.insertAfter(index, lat, lon, yaw);
  }

  bool Backend::captureAuto()
  {
    return recorder.autoCapture(node, trail);
  }

  void Backend::trailShift(int index, bool dir)
  {
    trail.shift(index, dir);
  }

  void Backend::trailEdit(int index, double lat, double lon, double orient)
  {
    trail.edit(index, lat, lon, orient);
  }

  void Backend::trailRemove(int index)
  {
    trail.remove(index);
  }

  void Backend::trailClear()
  {
    trail.clear();
  }

  void Backend::capture(int index)
  {
    recorder.capture(index, node, trail);
  }

  void Backend::mission_start()
  {
    mission_planner::Scheduler tmp(trail, node);
    scheduler = tmp;
  }

  void Backend::mission_pause()
  {
    scheduler.pause(node);
  }

  void Backend::mission_reverse()
  {
    scheduler.reverse(node);
  }

  void Backend::mission_abort()
  {
    scheduler.abort(node);
  }

  std::string Backend::getRobotPos()
  {
    std::ostringstream ostrings;
    std::string output;

    ostrings << std::setprecision(std::numeric_limits<double>::digits10);
    ostrings << node.getRobotLat() << ", ";
    ostrings << node.getRobotLon() << ", ";
    ostrings << node.getRobotYaw();

    output = ostrings.str();

    return output;
  }

  void Backend::captureReset()
  {
    recorder.reset();
  }

  std::string Backend::getTarget()
  {
    std::ostringstream ostrings;
    std::string output;

    if(scheduler.getTarget() < 0 || scheduler.getTarget() > trail.getSize() - 1)
    {
      return "No target / mission complete";
    }
    else
    {
      ostrings << std::setprecision(std::numeric_limits<double>::digits10);
      ostrings << (scheduler.getTarget() + 1) << ") ";
      ostrings << scheduler.getTargetLat() << ", ";
      ostrings << scheduler.getTargetLon() << ", ";
      ostrings << scheduler.getTargetOrient();
    }

    output = ostrings.str();
    return output;
  }

  int Backend::getProgress()
  {
    return scheduler.getProgress();
  }

  std::string Backend::getMissionState()
  {
    std::string output = "";

    switch(scheduler.getState())
    {
      case scheduler.ABORTED:
        output = "ABORTED";
        break;
      case scheduler.BEGIN:
        output = "BEGIN";
        break;
      case scheduler.FINISHED:
        output = "FINISHED";
        break;
      case scheduler.GOALACHIEVED:
        output = "GOAL ACHIEVED";
        break;
      case scheduler.GOALSENT:
        output = "GOAL SENT";
        break;
      case scheduler.NOTSTARTED:
        output = "NOT STARTED";
        break;
      case scheduler.PAUSED:
        output = "PAUSED";
        break;
      case scheduler.STOPPING_ROBOT:
        output = "STOPPING ROBOT";
        break;
      default:
        output = "Backend error: unknown scheduler state";
        break;
    }

    return output;
  }

  int Backend::trailSave(std::string dir)
  {
    return trail.save(dir);
  }

  int Backend::trailLoad(std::string dir)
  {
    return trail.load(dir);
  }

  bool Backend::waypointVisited(int index)
  {
    return trail.hasVisited(index);
  }
}
