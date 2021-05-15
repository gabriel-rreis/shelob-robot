# Shelob Hexapod

Shelob is a six-legged hexapod robot designed as a learning project. It runs mostly on Python code embedded in a Linux single board computer wrapped on the ROS framework. Currently on development.

## Installation

Make sure to have [ROS Foxy](rosdocs) and `colcon` installed.

```sh
virtualenv -p python3 .venv
source .venv/bin/activate
touch .venv/COLCON_IGNORE
python3 -m pip install -r requirements.txt
source /opt/ros2/foxy/setup.bash
colcon build
```

<!--
## Usage

A few motivating and useful examples of how your product can be used. Spice this up with code blocks and potentially more screenshots.

_For more examples and usage, please refer to the [Wiki][wiki]._

## Development setup

Describe how to install all development dependencies and how to run an automated test-suite of some kind. Potentially do this for multiple platforms.
-->

## Meta

Gabriel Reis – [@VaguelyRobotic](https://twitter.com/vaguelyrobotic) – [gabrielreis.info](gabrielreis.info)

Distributed under the GNU license. See `LICENSE` for more information.

<!-- Markdown link & img dfn's -->

[rosdocs]: https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools
