# Mission Qt

A Qt-based graphical desktop application for planning and executing autonomous
navigation missions for robots.

This ROS package acts as a front end for `mission_planner`, a ROS package which
provides a ROS server to which Mission Qt will interface with.

## Getting Started

1. Download and set up the `mission_planner` ROS package
2. Start the server (`missionsrv`): `roslaunch mission_planner missionsrv`
3. Start Mission Qt: `roslaunch mission_planner_qt mission_qt`
