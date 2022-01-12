# Mission Qt

A Qt-based graphical desktop application for planning and executing autonomous
navigation missions for robots.

This repository is a [ROS](https://wiki.ros.org/ROS/Introduction) package which
provides the application. The application acts as a client which interfaces with
the `missionsrv` ROS server provided by
[mission_planner](https://github.com/jackrgm/mission_planner).

## Work in Progress - Here be Dragons

Mission Qt is currently under development, being taken from an old personal
project and being redesigned, therefore the project code is currently in a
work-in-progress state and is not yet currently ready for release and public
use. However, you can view the current code in the
[dev](https://github.com/jackrgm/mission_planner_qt/tree/dev) branch. Changes
are rapid and unpredictable in this state until a first major version release.

## Getting Started

1. Download and set up the `mission_planner` ROS package
2. Start the server (`missionsrv`): `roslaunch mission_planner missionsrv`
3. Start Mission Qt: `roslaunch mission_planner_qt mission_qt`
