#include "mission_planner_qt/missiontask.h"

// NOTE deprecated class
    // was an attempt at creating a bridge that would allow the missionwindow
    // to either use qt_robot_tasks or to run as an independent window

MissionTask::MissionTask(qt_robot_tasks::robotTasksMainWindow* mw, int pos,
  QWidget* parent)
  : qt_robot_tasks::robotTask(mw, 'r', "&Mission Planner", pos, parent)
{
  // set up mission window
  misswin.setSignalSlots();
  misswin.setDefaults();
}

// TODO relearn this bit and cleanup
// TODO define message strings in class and use var in outputs
bool MissionTask::activate(ros::NodeHandle* nh, const std::string& rosNamespace) {
	//if (!activated_) { not finding robottask's variables :(
    std::cout << "Activating the Navigation task on namespace `"
    << rosNamespace << "'...\n";

    // TODO stuff to do on activation

    //activated_ = true;
    std::cout << "Navigation task activated" << "\n";
  //}

    return(true);
}

void MissionTask::deactivate() {
	//if (activated_) { not finding robottask's variables :(

	// TODO any housekeeping and deactivation steps

    std::cout << "Navigation task deactivated" << "\n";

    // TODO delete any relevant resources (e.g. topicnames) used
    //activated_ = false;
  //}
}
