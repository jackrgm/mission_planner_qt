#include <ros/ros.h>
#include <thread>

#include "mission_planner_qt/missionwindow.h"
//#include "robotTasksMainWindow.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mission_qt");
  //ros::NodeHandle nh;
  QApplication app(argc, argv);
  bool quit;

  // qt_robot_tasks is an integrated ui environment
  /*qt_robot_tasks::robotTasksMainWindow* qt_rt
    = new qt_robot_tasks::robotTasksMainWindow(&nh, NULL,
    "Robot Tasks");
  */

  // integrate mission gui into qt_robot_tasks
  //MissionWindow* mission = new MissionWindow(qt_rt, 1);
  //qt_rt->registerTask(mission);

  mission_planner_qt::MissionWindow mission(NULL);

  // TODO try dissecting [&]( and play about with this block
  // TODO does qt_robot_tasks need this? doesn't seem to work for me
  /*
  auto t = std::thread([&]() {
    //usleep(1000);
    ros::spin();
  });
  */

  // launch qt robot tasks with registered tasks
  //qt_rt->show();

  mission.show();

  // start gui control flow (should return before joining any open threads)
  int ret = app.exec();
  mission.close();
  //t.join();

  //delete qt_rt;
  //delete mission;
  return(ret);
}
